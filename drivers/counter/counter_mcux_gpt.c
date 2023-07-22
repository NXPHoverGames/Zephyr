/*
 * Copyright (c) 2019, Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_gpt

#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <fsl_gpt.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>

#define GPT_INST DT_INST(0, DT_DRV_COMPAT)
LOG_MODULE_REGISTER(mcux_gpt, CONFIG_COUNTER_LOG_LEVEL);

struct mcux_gpt_config {
	/* info must be first element */
	struct counter_config_info info;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	GPT_Type *base;
	clock_name_t clock_source;
};

struct mcux_gpt_data {
	counter_alarm_callback_t alarm_callback;
	counter_top_callback_t top_callback;
	counter_capture_callback_t capture_callback;
	void *alarm_user_data;
	void *top_user_data;
	void *capture_user_data;
};
/*
 * PPM decoder tuning parameters
 */
#  define PPM_MIN_PULSE_WIDTH    200    /**< minimum width of a valid first pulse */
#  define PPM_MAX_PULSE_WIDTH    600    /**< maximum width of a valid first pulse */
#  define PPM_MIN_CHANNEL_VALUE  800    /**< shortest valid channel signal */
#  define PPM_MAX_CHANNEL_VALUE  2200   /**< longest valid channel signal */
#  define PPM_MIN_START          2300   /**< shortest valid start gap (only 2nd part of pulse) */

/* decoded PPM buffer */

#  define PPM_MIN_CHANNELS       5
#  define PPM_MAX_CHANNELS       20

/** Number of same-sized frames required to 'lock' */

#  define PPM_CHANNEL_LOCK       4 /**< should be less than the input timeout */

/* Following data structures are creatd to be used by RC Input driver or any application */
uint16_t ppm_buffer[PPM_MAX_CHANNELS];
uint16_t ppm_frame_length = 0;
unsigned ppm_decoded_channels = 0;
uint64_t ppm_last_valid_decode = 0;
static uint32_t ppm_temp_buffer[PPM_MAX_CHANNELS];
/**********/

/** PPM decoder state machine */
struct {
        uint32_t        last_edge;      /**< last capture time */
        uint32_t        last_mark;      /**< last significant edge */
        uint32_t        frame_start;    /**< the frame width */
        unsigned        next_channel;   /**< next channel index */
        enum {
                UNSYNCH = 0,
                ARM,
                ACTIVE,
                INACTIVE
        } phase;
} ppm;

static void mcux_gpt_ppm_decode(const struct device *dev);

static int mcux_gpt_start(const struct device *dev)
{
	const struct mcux_gpt_config *config = dev->config;
        printk("\n[SUMIT] mcux_gpt_start \n");
	GPT_StartTimer(config->base);

	return 0;
}

static int mcux_gpt_stop(const struct device *dev)
{
	const struct mcux_gpt_config *config = dev->config;

	GPT_StopTimer(config->base);

	return 0;
}

static int mcux_gpt_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct mcux_gpt_config *config = dev->config;

	*ticks = GPT_GetCurrentTimerCount(config->base);
	return 0;
}

static int mcux_gpt_set_alarm(const struct device *dev, uint8_t chan_id,
			      const struct counter_alarm_cfg *alarm_cfg)
{
	const struct mcux_gpt_config *config = dev->config;
	struct mcux_gpt_data *data = dev->data;

	uint32_t current = GPT_GetCurrentTimerCount(config->base);
	uint32_t ticks = alarm_cfg->ticks;

	if (chan_id != 0) {
		LOG_ERR("Invalid channel id");
		return -EINVAL;
	}

	if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) == 0) {
		ticks += current;
	}

	if (data->alarm_callback) {
		return -EBUSY;
	}

	data->alarm_callback = alarm_cfg->callback;
	data->alarm_user_data = alarm_cfg->user_data;

	GPT_SetOutputCompareValue(config->base, kGPT_OutputCompare_Channel1,
				  ticks);
	GPT_EnableInterrupts(config->base, kGPT_OutputCompare1InterruptEnable);

	return 0;
}

static int mcux_gpt_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	const struct mcux_gpt_config *config = dev->config;
	struct mcux_gpt_data *data = dev->data;

	if (chan_id != 0) {
		LOG_ERR("Invalid channel id");
		return -EINVAL;
	}

	GPT_DisableInterrupts(config->base, kGPT_OutputCompare1InterruptEnable);
	data->alarm_callback = NULL;

	return 0;
}

// This block scans for a supported serial RC input and locks onto the first one found
// Scan for 500 msec, then switch protocol
typedef uint64_t        mcux_gpt_abstime;
const mcux_gpt_abstime rc_scan_max = 500; //In millisecond
#if 1

static mcux_gpt_abstime mcux_gpt_absolute_time(const struct device *dev)
{
	const struct mcux_gpt_config *config = dev->config;
	uint32_t now_ticks;
	mcux_gpt_abstime now_usec = 0;
        int err = 0;

	err = mcux_gpt_get_value(dev, &now_ticks);
	if (err) {
		printk("Failed to read counter value (err %d)", err);
		return 0;
	}

        now_usec = ((mcux_gpt_abstime)now_ticks * USEC_PER_SEC) / config->info.freq;
        return now_usec;
}

/**
 * Handle the PPM decoder state machine.
 */
static void mcux_gpt_ppm_decode(const struct device *dev)
{
	const struct mcux_gpt_config *config = dev->config;
        uint32_t count;
        uint32_t width;
        uint32_t interval;
        unsigned i;

        /* TODO: Need to pass the ICR channel value based on device tree
         * selection */
        count = GPT_GetInputCaptureValue(config->base, 1);

        /* how long since the last edge? - this handles counter wrapping implicitly. */
        width = count - ppm.last_edge;

        /*
         * if this looks like a start pulse, then push the last set of values
         * and reset the state machine
         */
        if (width >= PPM_MIN_START) {

                /*
                 * If the number of channels changes unexpectedly, we don't want
                 * to just immediately jump on the new count as it may be a result
                 * of noise or dropped edges.  Instead, take a few frames to settle.
                 */
                if (ppm.next_channel != ppm_decoded_channels) {
                        static unsigned new_channel_count;
                        static unsigned new_channel_holdoff;

                        if (new_channel_count != ppm.next_channel) {
                                /* start the lock counter for the new channel count */
                                new_channel_count = ppm.next_channel;
                                new_channel_holdoff = PPM_CHANNEL_LOCK;

                        } else if (new_channel_holdoff > 0) {
                                /* this frame matched the last one, decrement the lock counter */
                                new_channel_holdoff--;

                        } else {
                                /* we have seen PPM_CHANNEL_LOCK frames with the new count, accept it */
                                ppm_decoded_channels = new_channel_count;
                                new_channel_count = 0;
                        }

                } else {
                        /* frame channel count matches expected, let's use it */
                        if (ppm.next_channel >= PPM_MIN_CHANNELS) {
                                for (i = 0; i < ppm.next_channel; i++) {
                                        ppm_buffer[i] = ppm_temp_buffer[i];
                                }

                                ppm_last_valid_decode = mcux_gpt_absolute_time(dev);

                        }
                }

                /* reset for the next frame */
                ppm.next_channel = 0;

                /* next edge is the reference for the first channel */
                ppm.phase = ARM;

                ppm.last_edge = count;
                return;
        }

        switch (ppm.phase) {
        case UNSYNCH:
                /* we are waiting for a start pulse - nothing useful to do here */
                break;

        case ARM:

                /* we expect a pulse giving us the first mark */
                if (width < PPM_MIN_PULSE_WIDTH || width > PPM_MAX_PULSE_WIDTH) {
                        goto error;        /* pulse was too short or too long */
                }
                /* record the mark timing, expect an inactive edge */
                ppm.last_mark = ppm.last_edge;

                /* frame length is everything including the start gap */
                ppm_frame_length = (uint16_t)(ppm.last_edge - ppm.frame_start);
                ppm.frame_start = ppm.last_edge;
                ppm.phase = ACTIVE;
                break;

        case INACTIVE:

                /* we expect a short pulse */
                if (width < PPM_MIN_PULSE_WIDTH || width > PPM_MAX_PULSE_WIDTH) {
                        goto error;        /* pulse was too short or too long */
                }

                /* this edge is not interesting, but now we are ready for the next mark */
                ppm.phase = ACTIVE;
                break;

        case ACTIVE:
                /* determine the interval from the last mark */
                interval = count - ppm.last_mark;
                ppm.last_mark = count;

#if PPM_DEBUG
                ppm_pulse_history[ppm_pulse_next++] = interval;

                if (ppm_pulse_next >= EDGE_BUFFER_COUNT) {
                        ppm_pulse_next = 0;
                }

#endif
                /* if the mark-mark timing is out of bounds, abandon the frame */
                if ((interval < PPM_MIN_CHANNEL_VALUE) || (interval > PPM_MAX_CHANNEL_VALUE)) {
                        goto error;
                }

                /* if we have room to store the value, do so */
                if (ppm.next_channel < PPM_MAX_CHANNELS) {
                        ppm_temp_buffer[ppm.next_channel++] = interval;
                }

                ppm.phase = INACTIVE;
                break;

        }

        ppm.last_edge = count;
        return;

        /* the state machine is corrupted; reset it */

error:
        /* we don't like the state of the decoder, reset it and try again */
        ppm.phase = UNSYNCH;
        ppm_decoded_channels = 0;

}
#endif

void mcux_gpt_isr(const struct device *dev)
{
	const struct mcux_gpt_config *config = dev->config;
	struct mcux_gpt_data *data = dev->data;
	uint32_t current = GPT_GetCurrentTimerCount(config->base);
	uint32_t status;

	status =  GPT_GetStatusFlags(config->base, kGPT_OutputCompare1Flag |
				     kGPT_RollOverFlag | kGPT_InputCapture1Flag | kGPT_InputCapture2Flag);
        printk("\n[SUMIT] mcux_gpt_isr status = 0x%x\n",status);

	GPT_ClearStatusFlags(config->base, status);
	barrier_dsync_fence_full();

	if ((status & kGPT_OutputCompare1Flag) && data->alarm_callback) {
		GPT_DisableInterrupts(config->base,
				      kGPT_OutputCompare1InterruptEnable);
		counter_alarm_callback_t alarm_cb = data->alarm_callback;
		data->alarm_callback = NULL;
		alarm_cb(dev, 0, current, data->alarm_user_data);
	}

	if ((status & kGPT_RollOverFlag) && data->top_callback) {
		data->top_callback(dev, data->top_user_data);
	}

        if ((status & kGPT_InputCapture2Flag)) { // && data->capture_callback) {
                printk("\n[SUMIT] GPT1_CAPTURE2 Interrupt\n");
                mcux_gpt_ppm_decode(dev);
        }
}

static uint32_t mcux_gpt_get_pending_int(const struct device *dev)
{
	const struct mcux_gpt_config *config = dev->config;

	return GPT_GetStatusFlags(config->base, kGPT_OutputCompare1Flag |
                                  kGPT_InputCapture1Flag | kGPT_InputCapture2Flag);
}

static int mcux_gpt_set_top_value(const struct device *dev,
				  const struct counter_top_cfg *cfg)
{
	const struct mcux_gpt_config *config = dev->config;
	struct mcux_gpt_data *data = dev->data;

	if (cfg->ticks != config->info.max_top_value) {
		LOG_ERR("Wrap can only be set to 0x%x",
			config->info.max_top_value);
		return -ENOTSUP;
	}

	data->top_callback = cfg->callback;
	data->top_user_data = cfg->user_data;

	GPT_EnableInterrupts(config->base, kGPT_RollOverFlagInterruptEnable);

	return 0;
}

static int mcux_gpt_setup_capture(const struct device *dev,
				  const struct counter_capture_cfg *capture_cfg)
{
	const struct mcux_gpt_config *config = dev->config;
	struct mcux_gpt_data *data = dev->data;
        uint32_t mode = kGPT_InputOperation_Disabled;

	data->capture_callback = capture_cfg->callback;
	data->capture_user_data = capture_cfg->user_data;

        if(capture_cfg->flags & COUNTER_CAPTURE_CFG_OPERATION_MODE_RISING)
                mode = kGPT_InputOperation_RiseEdge;
        else if(capture_cfg->flags & COUNTER_CAPTURE_CFG_OPERATION_MODE_FALLING)
                mode = kGPT_InputOperation_FallEdge;
        else if(capture_cfg->flags & COUNTER_CAPTURE_CFG_OPERATION_MODE_BOTH)
                mode = kGPT_InputOperation_BothEdge;
        else
                mode = kGPT_InputOperation_Disabled;

        /* TODO: Decide Capture1 or 2 based on a board specific CONFIG or MACRO */
        GPT_SetInputOperationMode(config->base, kGPT_InputCapture_Channel1 | kGPT_InputCapture_Channel2, kGPT_InputOperation_BothEdge);

        /* TODO: Decide Capture1 or 2 based on a board specific CONFIG or MACRO */
	GPT_EnableInterrupts(config->base, kGPT_InputCapture1InterruptEnable|kGPT_InputCapture2InterruptEnable);
        printk("\nmcux_gpt_setup_capture\n");
	return 0;
}

static uint32_t mcux_gpt_get_top_value(const struct device *dev)
{
	const struct mcux_gpt_config *config = dev->config;

	return config->info.max_top_value;
}

static int mcux_gpt_init(const struct device *dev)
{
	const struct mcux_gpt_config *config = dev->config;
	gpt_config_t gptConfig;
	uint32_t clock_freq;

        if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	if (clock_control_get_rate(config->clock_dev, config->clock_subsys,
				   &clock_freq)) {
		return -EINVAL;
	}
        printk("\n[SUMIT]mcux_gpt_init CLK_DEV NAME =%s clk freq = %d config->info.frequency=%d\n",config->clock_dev->name, clock_freq, config->info.freq);
	/* Adjust divider to match expected freq */
	if (clock_freq % config->info.freq) {
		LOG_ERR("Cannot Adjust GPT freq to %u\n", config->info.freq);
		LOG_ERR("clock src is %u\n", clock_freq);
		return -EINVAL;
	}

	//GPT_StopTimer(config->base);
	GPT_GetDefaultConfig(&gptConfig);
	gptConfig.enableRunInStop = true;
	gptConfig.enableRunInWait = true;
	gptConfig.enableRunInDoze = true;
	gptConfig.enableFreeRun = true; /* Do not reset on compare */
	gptConfig.clockSource = kGPT_ClockSource_LowFreq;//kGPT_ClockSource_Periph;
	gptConfig.divider = clock_freq / config->info.freq;
	GPT_Init(config->base, &gptConfig);
	gptConfig.clockSource = kGPT_ClockSource_Periph;
        GPT_SetInputOperationMode(config->base, kGPT_InputCapture_Channel1 | kGPT_InputCapture_Channel2, kGPT_InputOperation_BothEdge);

        /* TODO: Decide Capture1 or 2 based on a board specific CONFIG or MACRO */
	GPT_EnableInterrupts(config->base, kGPT_InputCapture1InterruptEnable|kGPT_InputCapture2InterruptEnable);
	irq_enable(DT_IRQN(GPT_INST));
	GPT_StartTimer(config->base);

        printk("\n GPT Counter Driver Init Success BASE=0x%x\n",config->base);
        printk("\nGPT Reg=0x%x\n", *(config->base));
	return 0;
}

static const struct counter_driver_api mcux_gpt_driver_api = {
	.start = mcux_gpt_start,
	.stop = mcux_gpt_stop,
	.get_value = mcux_gpt_get_value,
	.set_alarm = mcux_gpt_set_alarm,
	.cancel_alarm = mcux_gpt_cancel_alarm,
	.set_top_value = mcux_gpt_set_top_value,
	.get_pending_int = mcux_gpt_get_pending_int,
	.get_top_value = mcux_gpt_get_top_value,
        .setup_capture = mcux_gpt_setup_capture,
};

#define GPT_DEVICE_INIT_MCUX(n)						\
	static struct mcux_gpt_data mcux_gpt_data_ ## n;		\
									\
	static const struct mcux_gpt_config mcux_gpt_config_ ## n = {	\
		.base = (void *)DT_INST_REG_ADDR(n),			\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),	\
		.clock_subsys =						\
			(clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),\
		.info = {						\
			.max_top_value = UINT32_MAX,			\
			.freq = DT_INST_PROP(n, gptfreq),           \
			.channels = 1,					\
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,		\
		},							\
	};								\
									\
	static int mcux_gpt_## n ##_init(const struct device *dev);	\
	DEVICE_DT_INST_DEFINE(n,					\
			    mcux_gpt_## n ##_init,			\
			    NULL,					\
			    &mcux_gpt_data_ ## n,			\
			    &mcux_gpt_config_ ## n,			\
			    POST_KERNEL,				\
			    CONFIG_COUNTER_INIT_PRIORITY,		\
			    &mcux_gpt_driver_api);			\
									\
	static int mcux_gpt_## n ##_init(const struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    mcux_gpt_isr, DEVICE_DT_INST_GET(n), 0);	\
		return mcux_gpt_init(dev);				\
	}								\

DT_INST_FOREACH_STATUS_OKAY(GPT_DEVICE_INIT_MCUX)
