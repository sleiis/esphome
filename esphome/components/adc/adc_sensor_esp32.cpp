#ifdef USE_ESP32

#include "adc_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace adc {

#if CONFIG_IDF_TARGET_ESP32
#define ADC1_CHAN0 ADC_CHANNEL_4
#define ADC1_CHAN1 ADC_CHANNEL_5
#else
#define ADC1_CHAN0 ADC_CHANNEL_0  // GPIO0
#define ADC1_CHAN1 ADC_CHANNEL_1  // GPIO1
#define ADC1_CHAN2 ADC_CHANNEL_2  // GPIO2
#define ADC1_CHAN3 ADC_CHANNEL_3  // GPIO3
#define ADC1_CHAN4 ADC_CHANNEL_4  // GPIO4
#define ADC1_CHAN5 ADC_CHANNEL_5  // GPIO5
#define ADC1_CHAN6 ADC_CHANNEL_6  // GPIO6
#endif

static const char *const TAG = "adc.esp32";

static const adc_bits_width_t ADC_WIDTH_MAX_SOC_BITS = static_cast<adc_bits_width_t>(ADC_WIDTH_MAX - 1);

#ifndef SOC_ADC_RTC_MAX_BITWIDTH
#if USE_ESP32_VARIANT_ESP32S2
static const int32_t SOC_ADC_RTC_MAX_BITWIDTH = 13;
#else
static const int32_t SOC_ADC_RTC_MAX_BITWIDTH = 12;
#endif  // USE_ESP32_VARIANT_ESP32S2
#endif  // SOC_ADC_RTC_MAX_BITWIDTH

static const int ADC_MAX = (1 << SOC_ADC_RTC_MAX_BITWIDTH) - 1;
static const int ADC_HALF = (1 << SOC_ADC_RTC_MAX_BITWIDTH) >> 1;

#if (SOC_ADC_PERIPH_NUM >= 2) && !CONFIG_IDF_TARGET_ESP32C3
/**
 * On ESP32C3, ADC2 is no longer supported, due to its HW limitation.
 * Search for errata on espressif website for more details.
 */
#define USE_ADC2 1
#endif

void ADCSensor::setup() {
  ESP_LOGCONFIG(TAG, "Initializing ADC");

  adc_oneshot_unit_init_cfg_t unit_cfg = {
      .unit_id = ADC_UNIT_1,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &this->adc_handle_));

  adc_oneshot_chan_cfg_t chan_cfg = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = this->attenuation_,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(this->adc_handle_, this->channel1_, &chan_cfg));

  this->calibration_available_ =
      adc_calibration_init(ADC_UNIT_1, this->channel1_, this->attenuation_, &this->cali_handle_);
}

void ADCSensor::dump_config() {
  LOG_SENSOR("", "ADC Sensor", this);
  LOG_PIN("  Pin: ", this->pin_);
  if (this->autorange_) {
    ESP_LOGCONFIG(TAG, "  Attenuation: auto");
  } else {
    switch (this->attenuation_) {
      case ADC_ATTEN_DB_0:
        ESP_LOGCONFIG(TAG, "  Attenuation: 0db");
        break;
      case ADC_ATTEN_DB_2_5:
        ESP_LOGCONFIG(TAG, "  Attenuation: 2.5db");
        break;
      case ADC_ATTEN_DB_6:
        ESP_LOGCONFIG(TAG, "  Attenuation: 6db");
        break;
      case ADC_ATTEN_DB_12_COMPAT:
        ESP_LOGCONFIG(TAG, "  Attenuation: 12db");
        break;
      default:  // This is to satisfy the unused ADC_ATTEN_MAX
        break;
    }
  }
  ESP_LOGCONFIG(TAG,
                "  Samples: %i\n"
                "  Sampling mode: %s",
                this->sample_count_, LOG_STR_ARG(sampling_mode_to_str(this->sampling_mode_)));
  LOG_UPDATE_INTERVAL(this);
}

float ADCSensor::sample() {

float ADCSensor::sample() {
    if (this->adc_handle_ == nullptr) {
      ESP_LOGE(TAG, "ADC not initialized");
      return NAN;
    }

    int raw = 0;
    int mv = 0;

    if (adc_oneshot_read(this->adc_handle_, this->channel1_, &raw) != ESP_OK) {
      ESP_LOGE(TAG, "ADC read failed");
      return NAN;
    }

    if (this->calibration_available_) {
      if (adc_cali_raw_to_voltage(this->cali_handle_, raw, &mv) != ESP_OK) {
        ESP_LOGW(TAG, "Calibration failed");
        return raw;  // fallback auf raw
      }
      return mv / 1000.0f;
    }

    return raw;  // falls keine Kalibrierung


 /* if (!this->autorange_) {
    auto aggr = Aggregator(this->sampling_mode_);

    for (uint8_t sample = 0; sample < this->sample_count_; sample++) {
      int raw = -1;
      if (this->channel1_ != ADC1_CHANNEL_MAX) {
        raw = adc1_get_raw(this->channel1_);
      } else if (this->channel2_ != ADC2_CHANNEL_MAX) {
        adc2_get_raw(this->channel2_, ADC_WIDTH_MAX_SOC_BITS, &raw);
      }
      if (raw == -1) {
        return NAN;
      }

      aggr.add_sample(raw);
    }
    if (this->output_raw_) {
      return aggr.aggregate();
    }
    uint32_t mv =
        esp_adc_cal_raw_to_voltage(aggr.aggregate(), &this->cal_characteristics_[(int32_t) this->attenuation_]);
    return mv / 1000.0f;
  }

  int raw12 = ADC_MAX, raw6 = ADC_MAX, raw2 = ADC_MAX, raw0 = ADC_MAX;

  if (this->channel1_ != ADC1_CHANNEL_MAX) {
    adc1_config_channel_atten(this->channel1_, ADC_ATTEN_DB_12_COMPAT);
    raw12 = adc1_get_raw(this->channel1_);
    if (raw12 < ADC_MAX) {
      adc1_config_channel_atten(this->channel1_, ADC_ATTEN_DB_6);
      raw6 = adc1_get_raw(this->channel1_);
      if (raw6 < ADC_MAX) {
        adc1_config_channel_atten(this->channel1_, ADC_ATTEN_DB_2_5);
        raw2 = adc1_get_raw(this->channel1_);
        if (raw2 < ADC_MAX) {
          adc1_config_channel_atten(this->channel1_, ADC_ATTEN_DB_0);
          raw0 = adc1_get_raw(this->channel1_);
        }
      }
    }
  } else if (this->channel2_ != ADC2_CHANNEL_MAX) {
    adc2_config_channel_atten(this->channel2_, ADC_ATTEN_DB_12_COMPAT);
    adc2_get_raw(this->channel2_, ADC_WIDTH_MAX_SOC_BITS, &raw12);
    if (raw12 < ADC_MAX) {
      adc2_config_channel_atten(this->channel2_, ADC_ATTEN_DB_6);
      adc2_get_raw(this->channel2_, ADC_WIDTH_MAX_SOC_BITS, &raw6);
      if (raw6 < ADC_MAX) {
        adc2_config_channel_atten(this->channel2_, ADC_ATTEN_DB_2_5);
        adc2_get_raw(this->channel2_, ADC_WIDTH_MAX_SOC_BITS, &raw2);
        if (raw2 < ADC_MAX) {
          adc2_config_channel_atten(this->channel2_, ADC_ATTEN_DB_0);
          adc2_get_raw(this->channel2_, ADC_WIDTH_MAX_SOC_BITS, &raw0);
        }
      }
    }
  }

  if (raw0 == -1 || raw2 == -1 || raw6 == -1 || raw12 == -1) {
    return NAN;
  }

  uint32_t mv12 = esp_adc_cal_raw_to_voltage(raw12, &this->cal_characteristics_[(int32_t) ADC_ATTEN_DB_12_COMPAT]);
  uint32_t mv6 = esp_adc_cal_raw_to_voltage(raw6, &this->cal_characteristics_[(int32_t) ADC_ATTEN_DB_6]);
  uint32_t mv2 = esp_adc_cal_raw_to_voltage(raw2, &this->cal_characteristics_[(int32_t) ADC_ATTEN_DB_2_5]);
  uint32_t mv0 = esp_adc_cal_raw_to_voltage(raw0, &this->cal_characteristics_[(int32_t) ADC_ATTEN_DB_0]);

  uint32_t c12 = std::min(raw12, ADC_HALF);
  uint32_t c6 = ADC_HALF - std::abs(raw6 - ADC_HALF);
  uint32_t c2 = ADC_HALF - std::abs(raw2 - ADC_HALF);
  uint32_t c0 = std::min(ADC_MAX - raw0, ADC_HALF);
  uint32_t csum = c12 + c6 + c2 + c0;

  uint32_t mv_scaled = (mv12 * c12) + (mv6 * c6) + (mv2 * c2) + (mv0 * c0);
  return mv_scaled / (float) (csum * 1000U);*/
}

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten,
                                 adc_cali_handle_t *out_handle) {
  adc_cali_handle_t handle = NULL;
  esp_err_t ret = ESP_FAIL;
  bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
      calibrated = true;
    }
  }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
      calibrated = true;
    }
  }
#endif

  *out_handle = handle;
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Calibration Success");
  } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
    ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
  } else {
    ESP_LOGE(TAG, "Invalid arg or no memory");
  }

  return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
  ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
  ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
#endif  // USE_ESP32


}  // namespace adc
}  // namespace esphome

#endif  // USE_ESP32
