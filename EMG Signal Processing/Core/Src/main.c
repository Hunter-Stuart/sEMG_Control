/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "bdma.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "jy901s.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VREFINT 1.2f  // stm32内部参考电压
#define FFT_POINT 256 // FFT取的点数
#define FS 2000.0     // 采样率
#define PAI 3.1415926f
#define F_MIN 20.0f           // 有效频率下限
#define F_MAX 500.0f          // 有效频率上限

#define FEATURE1 20           //通道一判据
#define FEATURE2 20           //通道二判据
#define FEATURE_BUF 5         //feature_buf缓冲区大小
#define ADC_BUF_LEN FFT_POINT // DMA循环缓冲区长度=FFT点数，避免数据覆盖
#define IIR_STAGES 51         // 二阶节数量
float iir_sos_coeffs[IIR_STAGES * 5] = {
    1.0f, 0.0f, -1.0f, 0.8693764321864266f, -0.9536568122771949f,
    1.0f, 0.0f, -1.0f, 1.9864247945818152f, -0.9950821659378192f,
    1.0f, 0.0f, -1.0f, 0.8321927690588344f, -0.8672413716423795f,
    1.0f, 0.0f, -1.0f, 1.9766845753697753f, -0.9853141721013935f,
    1.0f, 0.0f, -1.0f, 0.7995116982474844f, -0.7884588794031837f,
    1.0f, 0.0f, -1.0f, 1.9670113874654054f, -0.9756280179750199f,
    1.0f, 0.0f, -1.0f, 0.770848011898446f,  -0.716535449340347f,
    1.0f, 0.0f, -1.0f, 1.957395916219848f,  -0.9660142440349393f,
    1.0f, 0.0f, -1.0f, 0.7457813082643308f, -0.6507974014896779f,
    1.0f, 0.0f, -1.0f, 1.9478296799657244f, -0.9564642770692043f,
    1.0f, 0.0f, -1.0f, 0.7239463586093481f, -0.5906576596226908f,
    1.0f, 0.0f, -1.0f, 1.9383052735176061f, -0.9469706669682229f,
    1.0f, 0.0f, -1.0f, 0.7050248651784609f, -0.5356041792878115f,
    1.0f, 0.0f, -1.0f, 1.9288166627628121f, -0.9375273720718309f,
    1.0f, 0.0f, -1.0f, 1.9193595525535816f, -0.9281301144804434f,
    1.0f, 0.0f, -1.0f, 0.6887383685610393f, -0.4851901183749884f,
    1.0f, 0.0f, -1.0f, 1.9099318548941653f, -0.9187768311736749f,
    1.0f, 0.0f, -1.0f, 0.6748420878986934f, -0.4390254967169763f,
    1.0f, 0.0f, -1.0f, 1.9005342908547407f, -0.9094682527833103f,
    1.0f, 0.0f, -1.0f, 0.6631194987885907f, -0.3967701250049715f,
    1.0f, 0.0f, -1.0f, 1.8911711678173693f, -0.9002086494961432f,
    1.0f, 0.0f, -1.0f, 0.6533774704597843f, -0.3581276142251006f,
    1.0f, 0.0f, -1.0f, 1.8818513833778256f, -0.8910067926150773f,
    1.0f, 0.0f, -1.0f, 0.6454417958013445f, -0.3228403039023611f,
    1.0f, 0.0f, -1.0f, 1.8725897176605484f, -0.8818771899701817f,
    1.0f, 0.0f, -1.0f, 0.6391529560301001f, -0.2906849701907506f,
    1.0f, 0.0f, -1.0f, 1.8634084847713721f, -0.872841661545082f,
    1.0f, 0.0f, -1.0f, 0.6343619686491586f, -0.2614691930128323f,
    1.0f, 0.0f, -1.0f, 1.8543396168114308f, -0.8639313237794735f,
    1.0f, 0.0f, -1.0f, 0.6309261780014019f, -0.2350282748009753f,
    1.0f, 0.0f, -1.0f, 1.8454272407358796f, -0.8551890379230934f,
    1.0f, 0.0f, -1.0f, 0.6287048720741699f, -0.2112226116480437f,
    1.0f, 0.0f, -1.0f, 1.8367307617996442f, -0.8466723328984768f,
    1.0f, 0.0f, -1.0f, 0.627554665149167f,  -0.1899354205654527f,
    1.0f, 0.0f, -1.0f, 1.828328357904653f,  -0.8384567085004707f,
    1.0f, 0.0f, -1.0f, 0.6273247031893548f, -0.1710707240891596f,
    1.0f, 0.0f, -1.0f, 1.8203205733050356f, -0.8306390193629837f,
    1.0f, 0.0f, -1.0f, 0.6278519713113898f, -0.154551486646574f,
    1.0f, 0.0f, -1.0f, 1.8128333273942805f, -0.8233402866066106f,
    1.0f, 0.0f, -1.0f, 0.6289573609707806f, -0.1403177890563573f,
    1.0f, 0.0f, -1.0f, 1.8060191003433543f, -0.8167067601196332f,
    1.0f, 0.0f, -1.0f, 0.6304437131227081f, -0.1283249253220003f,
    1.0f, 0.0f, -1.0f, 1.8000544068476896f, -0.8109074411314436f,
    1.0f, 0.0f, -1.0f, 0.6320977081055255f, -0.11854132175107f,
    1.0f, 0.0f, -1.0f, 1.7951312534340129f, -0.8061258863773285f,
    1.0f, 0.0f, -1.0f, 0.6336978921996432f, -0.110946228356801f,
    1.0f, 0.0f, -1.0f, 1.791440768735733f,  -0.8025445900934733f,
    1.0f, 0.0f, -1.0f, 0.6350306397948811f, -0.1055272281685702f,
    1.0f, 0.0f, -1.0f, 1.789149387958497f,  -0.800322321401026f,
    1.0f, 0.0f, -1.0f, 0.6359136608599436f, -0.1022777433785706f,
    1.0f, 0.0f, -1.0f, 1.2122973192286475f, -0.284450719896452f};

const float iir_scale_vals[IIR_STAGES] = {
    0.4804150143471112f, 0.4804150143471112f, 0.4686281482944881f,
    0.4686281482944881f, 0.4577285824460943f, 0.4577285824460943f,
    0.4476494583907268f, 0.4476494583907268f, 0.4383307501573455f,
    0.4383307501573455f, 0.4297185698995845f, 0.4297185698995845f,
    0.421764541012795f,  0.421764541012795f,  0.4144252363555915f,
    0.4144252363555915f, 0.407661677559951f,  0.407661677559951f,
    0.401438890604339f,  0.401438890604339f,  0.3957255125638296f,
    0.3957255125638296f, 0.3904934445236531f, 0.3904934445236531f,
    0.3857175459074069f, 0.3857175459074069f, 0.3813753658378775f,
    0.3813753658378775f, 0.3774469075598798f, 0.3774469075598798f,
    0.3739144223755339f, 0.3739144223755339f, 0.3707622299521401f,
    0.3707622299521401f, 0.3679765622498276f, 0.3679765622498276f,
    0.3655454286749744f, 0.3655454286749744f, 0.363458500394341f,
    0.363458500394341f,  0.3617070120445879f, 0.3617070120445879f,
    0.3602836793443729f, 0.3602836793443729f, 0.359182631364359f,
    0.359182631364359f,  0.35839935643738f,   0.35839935643738f,
    0.3579306609000651f, 0.3579306609000651f, 0.357774640051774f};

float iir_scale_value=1;

// 预处理后的IIR系数（包含增益），格式：[b0*g, b1*g, b2*g, a1, a2]
float scaled_iir_coeffs[IIR_STAGES * 5];

// 陷波滤波器系数（MATLAB生成的直接II型滤波器）
// 单个二阶节，用于FFT前的信号处理
#define NOTCH_STAGES 1 // 陷波滤波器二阶节数量
const float notch_coeffs[NOTCH_STAGES * 5] = {
    // b0, b1, b2, a1, a2
    0.999334363637892630904957513848785310984f,
    -1.974061798642416265536780883849132806063f,
    0.999334363637892630904957513848785310984f,
    1.974061798642416265536780883849132806063f, // 注意：MATLAB的a1取反
    -0.998668727275785261809915027697570621967f // 注意：MATLAB的a2取反
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
arm_biquad_casd_df1_inst_f32 iir_filter1;
arm_biquad_casd_df1_inst_f32 iir_filter2;
float emg1_filter_state[13 * IIR_STAGES] = {0}; // EMG1滤波状态
float emg2_filter_state[13 * IIR_STAGES] = {0}; // EMG2滤波状态
float emg1_filtered[FFT_POINT] ={0};          // EMG1滤波后数据
float emg2_filtered[FFT_POINT] = {0};          // EMG2滤波后数据

// 陷波滤波器实例和状态
arm_biquad_casd_df1_inst_f32 notch_filter1;
arm_biquad_casd_df1_inst_f32 notch_filter2;
float emg1_notch_state[13 * NOTCH_STAGES] = {0}; // EMG1陷波滤波状态
float emg2_notch_state[13 * NOTCH_STAGES] = {0}; // EMG2陷波滤波状态
float emg1_notch_filtered[FFT_POINT] = {0};     // EMG1陷波滤波后数据
float emg2_notch_filtered[FFT_POINT] = {0};     // EMG2陷波滤波后数据

// 汉明窗缓冲区（使用arm_hamming_f32函数生成）
float hamming_window[FFT_POINT] = {0}; // 汉明窗系数
float hamming_norm = 1.0f;             // 汉明窗归一化因子

uint16_t fft_buf_count = 0; // FFT缓冲区填充计数（0/64/128）

extern arm_cfft_instance_f32 arm_cfft_sR_f32_len128;
extern arm_cfft_instance_f32 arm_cfft_sR_f32_len64;
extern arm_cfft_instance_f32 arm_cfft_sR_f32_len256;

// 手势枚举定义
typedef enum {
  GESTURE_NONE, // 无手势
  GESTURE_FIST, // 握拳
  GESTURE_RIGHT, //
  GESTURE_LEFT //
} Gesture_TypeDef;

// ADC数据缓冲区
uint32_t adc_dual_buf[ADC_BUF_LEN]
    __attribute__((aligned(32))); // 双同步ADC1+ADC2数据
uint32_t adc3_buf[ADC_BUF_LEN] __attribute__((aligned(32)));   // ADC3（VREFINT）数据

// 采样同步标志（为半完成和完成回调分别设置独立标志）
// 半完成回调标志
uint8_t adc_dual_half_done = 0; // 双同步ADC半完成标志
uint8_t adc3_half_done = 0;     // ADC3半完成标志
// 完成回调标志
uint8_t adc_dual_cplt_done = 0; // 双同步ADC完成标志
uint8_t adc3_cplt_done = 0;     // ADC3完成标志

uint32_t values[3];               // ADC读取原始值
float vrefint = 0;                // 校准后参考电压
float voltage1 = 0, voltage2 = 0; // 两路肌电信号

// FFT相关缓冲区 - 32字节对齐以优化CMSIS-DSP性能
float emg1_buf[FFT_POINT]
    __attribute__((aligned(32))) = {0}; // EMG1的256点电压缓冲区
float emg2_buf[FFT_POINT]
    __attribute__((aligned(32))) = {0}; // EMG2的256点电压缓冲区
float fft1_in[2 * FFT_POINT]
    __attribute__((aligned(32))) = {0}; // 浮点FFT输入（实部+虚部，长度2*N）
float fft2_in[2 * FFT_POINT] __attribute__((aligned(32))) = {0};
float fft1_mag[FFT_POINT] __attribute__((aligned(32))) = {0}; // FFT幅值输出
float fft2_mag[FFT_POINT] __attribute__((aligned(32))) = {0};

// 提取到的有效频段特征值
float feature_buf1[FEATURE_BUF]={0};
float feature_buf2[FEATURE_BUF]={0};
float feat1 = 0, feat2 = 0;
Gesture_TypeDef current_gesture = GESTURE_NONE; // 当前识别的手势
char message[100] = "";

float roll=0, pitch=0, yaw=0;       // 姿态角
float acc_x, acc_y, acc_z;    // 加速度
float gyro_x, gyro_y, gyro_z; // 角速度
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void calc_fft(float *sample_buf, float *fft_in, float *fft_mag, uint16_t N) {
  // 构造FFT输入并应用汉明窗
  for (uint16_t n = 0; n < N; n++) {
    // 应用汉明窗以减少频谱泄漏
    fft_in[2 * n] = sample_buf[n] * hamming_window[n];
    fft_in[2 * n + 1] = 0.0f;
  }
  // 初始化并执行FFT
  arm_cfft_instance_f32 *fft_inst;
  switch (N) {
  case 128:
    fft_inst = &arm_cfft_sR_f32_len128;
    break;
  case 64:
    fft_inst = &arm_cfft_sR_f32_len64;
    break;
  case 256:
    fft_inst = &arm_cfft_sR_f32_len256;
    break;
  default:
    while (1)
      ; // 非法点数，卡死报错
  }
  arm_cfft_f32(fft_inst, fft_in, 0, 1);
  // 计算幅值
  arm_cmplx_mag_f32(fft_in, fft_mag, N);

  // 应用汉明窗归一化因子，补偿窗函数引起的能量损失
  for (uint16_t k = 0; k < N; k++) {
    fft_mag[k] *= hamming_norm;
  }
}

// 提取有效频段的功率谱密度（PSD）特征值
// 功率谱 = 幅值平方，反映频段能量
float extract_feature(float *fft_mag, uint16_t N, float fs, float f_min,
                      float f_max) {
  float sum_power = 0.0f; // 累加功率而非幅值
  // 计算有效频率对应的FFT索引范围
  uint16_t k_min = (uint16_t)(f_min * N / fs);
  uint16_t k_max = (uint16_t)(f_max * N / fs);
  // 边界保护（避免索引越界）
  k_max = (k_max >= N) ? (N - 1) : k_max;
  // 累加有效频段的功率谱密度（幅值平方）
  for (uint16_t k = k_min; k <= k_max; k++) {
    // 功率 = 幅值^2，归一化可选：/N 或 /(N^2)
    float power = fft_mag[k] * fft_mag[k];
    sum_power += power;
  }
  // 返回平均功率密度（可选归一化）
  return sum_power / (float)(k_max - k_min + 1);
}

// 判断手势，待修改
Gesture_TypeDef recognize_gesture(float feat1, float feat2) {
	static int i=0;
	feature_buf1[i]=feat1;
	feature_buf2[i]=feat2;
	i++;
	if((i+1)%FEATURE_BUF==0)
	{
		float sum1=0,sum2=0;
		for(int j=0;j<=i;j++)
		{
			sum1+=feature_buf1[j];
			sum2+=feature_buf2[j];
		}
		sum1=sum1/FEATURE_BUF;
		sum2=sum2/FEATURE_BUF;
		if(sum1>FEATURE1&&sum2>FEATURE2)
		{
			current_gesture=GESTURE_FIST;
		}
		if(sum1>FEATURE1&&sum2<FEATURE2)
		{
			current_gesture=GESTURE_RIGHT;
		}
		if(sum1<FEATURE1&&sum2>FEATURE2)
		{
			current_gesture=GESTURE_LEFT;
		}
		if(sum1<FEATURE1&&sum2<FEATURE2)
		{
			current_gesture=GESTURE_NONE;
		}
		i=0;
	}
  return current_gesture;
}

// 自定义汉明窗初始化函数
void hamming_window_init(void) {
  // 生成汉明窗系数: w[n] = 0.54 - 0.46 * cos(2π * n / (N - 1))
  for (uint16_t n = 0; n < FFT_POINT; n++) {
    float angle = 2.0f * PAI * n / (FFT_POINT - 1);
    hamming_window[n] = 0.54f - 0.46f * arm_cos_f32(angle);
  }

  // 计算归一化因子（用于补偿窗函数引起的能量损失）
  float win_sum = 0.0f;
  for (uint16_t n = 0; n < FFT_POINT; n++) {
    win_sum += hamming_window[n];
  }
  hamming_norm = 2.0f / win_sum;
}

// IIR系数预处理函数：将增益系数应用到SOS系数中，并修正符号约定
// CMSIS-DSP期望格式：[b0, b1, b2, -a1, -a2]
// 如果原始系数来自SciPy等标准工具，需要对a1和a2取反
void iir_coeffs_preprocess(void) {
  for (uint16_t i = 0; i < IIR_STAGES; i++) {
    // b0, b1, b2 乘以对应的增益系数
    scaled_iir_coeffs[i * 5 + 0] =
        iir_sos_coeffs[i * 5 + 0];// * iir_scale_vals[i];
    scaled_iir_coeffs[i * 5 + 1] =
        iir_sos_coeffs[i * 5 + 1];// * iir_scale_vals[i];
    scaled_iir_coeffs[i * 5 + 2] =
        iir_sos_coeffs[i * 5 + 2];// * iir_scale_vals[i];
    // ⚠️ 符号约定修正：CMSIS-DSP需要 -a1 和 -a2（取反）
    // 差分方程：y[n] = b0*x[n] + ... + a1*y[n-1] + a2*y[n-2]
    scaled_iir_coeffs[i * 5 + 3] = -iir_sos_coeffs[i * 5 + 3];
    scaled_iir_coeffs[i * 5 + 4] = -iir_sos_coeffs[i * 5 + 4];
  }
}

// IIR滤波函数：对整帧时域数据滤波
void iir_filter_frame(float *input, float *output, uint16_t len,
                      arm_biquad_casd_df1_inst_f32 *filter) {
  // CMSIS-DSP函数：输入数组、输出数组、长度、滤波器实例
  arm_biquad_cascade_df1_f32(filter, input, output, len);
}

/**
 * @brief  计算重心频率（Frequency Center of Gravity, FCG）
 *         功率加权的平均频率，反映EMG能量集中的频率点
 * @param  fft_mag: FFT幅值数组（输出自arm_cmplx_mag_f32）
 * @param  N: FFT点数（如256）
 * @param  fs: 采样率（如2000.0Hz）
 * @param  f_min: 有效频率下限（如20.0Hz）
 * @param  f_max: 有效频率上限（如500.0Hz）
 * @retval 重心频率值（Hz），出错时返回0.0
 */
float calc_frequency_cg(float *fft_mag, uint16_t N, float fs, float f_min, float f_max)
{
    // 1. 计算有效频率对应的FFT索引范围
    uint16_t k_min = (uint16_t)(f_min * N / fs);
    uint16_t k_max = (uint16_t)(f_max * N / fs);
    // 边界保护：避免索引越界
    k_min = k_min < 0 ? 0 : k_min;
    k_max = k_max >= N ? (N - 1) : k_max;
    if (k_min > k_max) return 0.0f;

    // 2. 累加功率和、频率-功率乘积和
    float sum_power = 0.0f;       // 分母：ΣP(k)
    float sum_freq_power = 0.0f;  // 分子：Σ(f_k * P(k))
    for (uint16_t k = k_min; k <= k_max; k++)
    {
        float f_k = (float)k * fs / N;  // 当前索引对应的实际频率
        float p_k = fft_mag[k] * fft_mag[k];  // 功率谱密度P(k)=|FFT(k)|²
        sum_power += p_k;
        sum_freq_power += f_k * p_k;
    }

    // 3. 避免除以0错误
    if (sum_power < 1e-6f)  // 功率和接近0时返回0
    {
        return 0.0f;
    }

    // 4. 计算重心频率
    return sum_freq_power / sum_power;
}

/**
 * @brief  计算平均频率（Mean Frequency, MF）
 *         有效频率范围内的算术平均频率（与FCG的功率加权区分）
 * @param  fft_mag: FFT幅值数组（仅用于确定有效范围，无功率加权）
 * @param  N: FFT点数（如256）
 * @param  fs: 采样率（如2000.0Hz）
 * @param  f_min: 有效频率下限（如20.0Hz）
 * @param  f_max: 有效频率上限（如500.0Hz）
 * @retval 平均频率值（Hz），出错时返回0.0
 */
float calc_mean_frequency(float *fft_mag, uint16_t N, float fs, float f_min, float f_max)
{
    // 1. 计算有效频率对应的FFT索引范围
    uint16_t k_min = (uint16_t)(f_min * N / fs);
    uint16_t k_max = (uint16_t)(f_max * N / fs);
    // 边界保护：避免索引越界
    k_min = k_min < 0 ? 0 : k_min;
    k_max = k_max >= N ? (N - 1) : k_max;
    if (k_min > k_max) return 0.0f;

    // 2. 累加有效频率的和
    float sum_freq = 0.0f;
    uint16_t m = 0;  // 有效频率点数量
    for (uint16_t k = k_min; k <= k_max; k++)
    {
        float f_k = (float)k * fs / N;  // 当前索引对应的实际频率
        sum_freq += f_k;
        m++;
    }

    // 3. 避免除以0错误
    if (m == 0)
    {
        return 0.0f;
    }

    // 4. 计算算术平均频率
    return sum_freq / (float)m;
}

/**
 * @brief  计算频率均方根（Root Mean Square Frequency, RMSF）
 *         功率加权的频率均方根
 * @param  fft_mag: FFT幅值数组（输出自arm_cmplx_mag_f32）
 * @param  N: FFT点数（如256）
 * @param  fs: 采样率（如2000.0Hz）
 * @param  f_min: 有效频率下限（如20.0Hz）
 * @param  f_max: 有效频率上限（如500.0Hz）
 * @retval 频率均方根值（Hz），出错时返回0.0
 */
float calc_rms_frequency(float *fft_mag, uint16_t N, float fs, float f_min, float f_max)
{
    // 1. 计算有效频率对应的FFT索引范围
    uint16_t k_min = (uint16_t)(f_min * N / fs);
    uint16_t k_max = (uint16_t)(f_max * N / fs);
    // 边界保护：避免索引越界
    k_min = k_min < 0 ? 0 : k_min;
    k_max = k_max >= N ? (N - 1) : k_max;
    if (k_min > k_max) return 0.0f;

    // 2. 累加功率和、频率平方-功率乘积和
    float sum_power = 0.0f;        // 分母：ΣP(k)
    float sum_freq2_power = 0.0f;  // 分子：Σ(f_k² * P(k))
    for (uint16_t k = k_min; k <= k_max; k++)
    {
        float f_k = (float)k * fs / N;  // 当前索引对应的实际频率
        float p_k = fft_mag[k] * fft_mag[k];  // 功率谱密度P(k)=|FFT(k)|²
        sum_power += p_k;
        sum_freq2_power += (f_k * f_k) * p_k;
    }

    // 3. 避免除以0错误
    if (sum_power < 1e-6f)
    {
        return 0.0f;
    }

    // 4. 计算频率均方根
    float rms_freq = sum_freq2_power / sum_power;
    return sqrt(rms_freq);  // 使用CMSIS-DSP的平方根函数（更高效）
}

/**
 * @brief  计算频率标准差（Frequency Standard Deviation, FSD）
 *         功率加权的频率相对于重心频率的离散程度
 * @param  fft_mag: FFT幅值数组（输出自arm_cmplx_mag_f32）
 * @param  N: FFT点数（如256）
 * @param  fs: 采样率（如2000.0Hz）
 * @param  f_min: 有效频率下限（如20.0Hz）
 * @param  f_max: 有效频率上限（如500.0Hz）
 * @retval 频率标准差值（Hz），出错时返回0.0
 */
float calc_freq_standard_deviation(float *fft_mag, uint16_t N, float fs, float f_min, float f_max)
{
    // 1. 先计算重心频率（FCG）
    float fcg = calc_frequency_cg(fft_mag, N, fs, f_min, f_max);
    if (fcg < 1e-6f)  // FCG计算失败时返回0
    {
        return 0.0f;
    }

    // 2. 计算有效频率对应的FFT索引范围
    uint16_t k_min = (uint16_t)(f_min * N / fs);
    uint16_t k_max = (uint16_t)(f_max * N / fs);
    // 边界保护：避免索引越界
    k_min = k_min < 0 ? 0 : k_min;
    k_max = k_max >= N ? (N - 1) : k_max;
    if (k_min > k_max) return 0.0f;

    // 3. 累加功率和、(f_k - FCG)²-功率乘积和
    float sum_power = 0.0f;                     // 分母：ΣP(k)
    float sum_freq_diff2_power = 0.0f;         // 分子：Σ((f_k - FCG)² * P(k))
    for (uint16_t k = k_min; k <= k_max; k++)
    {
        float f_k = (float)k * fs / N;  // 当前索引对应的实际频率
        float p_k = fft_mag[k] * fft_mag[k];  // 功率谱密度P(k)=|FFT(k)|²
        float freq_diff = f_k - fcg;
        sum_power += p_k;
        sum_freq_diff2_power += (freq_diff * freq_diff) * p_k;
    }

    // 4. 避免除以0错误
    if (sum_power < 1e-6f)
    {
        return 0.0f;
    }

    // 5. 计算频率标准差
    float fsd = sum_freq_diff2_power / sum_power;
    return sqrt(fsd);  // 使用CMSIS-DSP的平方根函数（更高效）
}

// 处理DMA缓冲区的半帧数据（双缓冲模式）
// buf_start: 缓冲区起始索引（0或64）
// len: 处理长度（通常为64）
void process_adc_half_buffer(uint16_t buf_start, uint16_t len) {
  // 边界检查
  if (buf_start + len > ADC_BUF_LEN)
    return;

  // 处理半帧数据
  for (uint16_t i = 0; i < len; i++) {
    uint16_t buf_idx = buf_start + i;     // DMA缓冲区索引
    uint16_t fft_idx = fft_buf_count + i; // FFT缓冲区索引

    // 1. 解析双同步ADC数据（低16=ADC1，高16=ADC2）
    uint32_t dual_data = adc_dual_buf[buf_idx];
    uint16_t adc1_val = (uint16_t)(dual_data & 0xFFFF); // 低16位=ADC1
    uint16_t adc2_val = (uint16_t)(dual_data >> 16);    // 高16位=ADC2
    uint16_t adc3_val = adc3_buf[buf_idx];              // ADC3数据

    // 2. 计算校准后电压（VREFINT校准）
    float vrefint_temp =
        VREFINT * (4095.0f / (float)adc3_val); // 12位ADC满量程4095

    vrefint_temp =3.3;

    float voltage1_temp = ((float)adc1_val / 4095.0f) * vrefint_temp;
    float voltage2_temp = ((float)adc2_val / 4095.0f) * vrefint_temp;

    // 3. 存储到EMG缓冲区（用于后续FFT）
    if (fft_idx < FFT_POINT) // 额外的边界保护
    {
      emg1_buf[fft_idx] = voltage1_temp;
      emg2_buf[fft_idx] = voltage2_temp;
    }
  }

  // 4. 更新FFT缓冲区计数
  fft_buf_count += len;

  // 5. 满128点触发FFT和滤波
  if (fft_buf_count >= FFT_POINT) {
    // IIR滤波
	  arm_biquad_cascade_df1_f32(&iir_filter1, emg1_buf, emg1_filtered, FFT_POINT);
	  arm_biquad_cascade_df1_f32(&iir_filter2, emg2_buf, emg2_filtered, FFT_POINT);
    //iir_filter_frame(emg1_buf, emg1_filtered, FFT_POINT, &iir_filter1);
    //iir_filter_frame(emg2_buf, emg2_filtered, FFT_POINT, &iir_filter2);
	  for(int i=0;i<256;i++)
	      {
	      	emg1_filtered[i]=emg1_filtered[i]*(iir_scale_value*1000);
	      	emg2_filtered[i]=emg2_filtered[i]*(iir_scale_value*1000);
	      }
    // 陷波滤波（在IIR滤波之后，FFT之前）
    iir_filter_frame(emg1_filtered, emg1_notch_filtered, FFT_POINT,
                     &notch_filter1);
    iir_filter_frame(emg2_filtered, emg2_notch_filtered, FFT_POINT,
                     &notch_filter2);

    // FFT运算（使用陷波滤波后的数据）
    calc_fft(emg1_notch_filtered, fft1_in, fft1_mag,
             FFT_POINT); // EMG1 陷波滤波后FFT
    calc_fft(emg2_notch_filtered, fft2_in, fft2_mag,
             FFT_POINT); // EMG2 陷波滤波后FFT

    feat1=extract_feature(fft1_mag, FFT_POINT, FS, F_MIN, F_MAX);
    feat2=extract_feature(fft2_mag, FFT_POINT, FS, F_MIN, F_MAX);
    sprintf(message,"%d",recognize_gesture(feat1, feat2));

    //HAL_UART_Transmit(&huart2, (uint8_t*)"1", sizeof("1"),100);
    //sprintf(message,"%.2f",fft1_mag[255]);
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
    //HAL_UART_Transmit(&huart2, (uint8_t*)"\n", strlen("\n"), 100);
    //HAL_UART_Transmit(&huart2, (uint8_t*)"success", sizeof("success"), 100);

    ///////

    // 重置计数+清空缓冲区（准备下一轮采样）
    fft_buf_count = 0;
    memset(emg1_buf, 0, sizeof(emg1_buf));
    memset(emg2_buf, 0, sizeof(emg2_buf));
    // 注意:同步标志在各自回调函数中重置,此处不需要
  }
}

// DMA传输完成回调(处理缓冲区后半部分:索引128-255)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  // ✅ 同步等待 ADC1 和 ADC3 的 DMA 都完成后再处理
  // 避免 ADC1 先完成时 ADC3 数据还未写入的问题
	//HAL_UART_Transmit(&huart2, (uint8_t*)"success", sizeof("success"), 100);
  if (hadc == &hadc1) {
    adc_dual_cplt_done = 1; // 使用完成回调专用标志
  }
  if (hadc == &hadc3) {
    adc3_cplt_done = 1; // 使用完成回调专用标志
  }
  // 两个 DMA 都完成后才处理缓冲区后半部分(128-255)

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //if (adc_dual_cplt_done == 1 && adc3_cplt_done == 1)
  if (adc_dual_cplt_done == 1 )
  {
    process_adc_half_buffer(FFT_POINT / 2, FFT_POINT / 2);
    // ✅ 立即重置标志,为下次回调做准备
    adc_dual_cplt_done = 0;
    adc3_cplt_done = 0;
  }
}

// DMA传输半完成回调(处理缓冲区前半部分:索引0-127)
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  // ✅ 同步等待 ADC1 和 ADC3 的 DMA 都半完成后再处理
  // 避免 ADC1 先半完成时 ADC3 数据还未写入的问题
	//HAL_UART_Transmit(&huart2, (uint8_t*)"success", sizeof("success"), 100);
  if (hadc == &hadc1) {
	  //HAL_UART_Transmit(&huart2, (uint8_t*)"success", sizeof("success"), 100);
    adc_dual_half_done = 1; // 使用半完成回调专用标志
  }
  if (hadc == &hadc3) {
	  //HAL_UART_Transmit(&huart2, (uint8_t*)"success", sizeof("success"), 100);
    adc3_half_done = 1; // 使用半完成回调专用标志
  }
  // 两个 DMA 都半完成后才处理缓冲区前半部分(0-127)
  //if (adc_dual_half_done == 1 && adc3_half_done == 1)
  if (adc_dual_half_done == 1)
  {
	  //HAL_UART_Transmit(&huart2, (uint8_t*)"success", sizeof("success"), 100);
    process_adc_half_buffer(0, FFT_POINT / 2);
    // ✅ 立即重置标志,为下次回调做准备
    adc_dual_half_done = 0;
    adc3_half_done = 0;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_ADC3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  //JY901S_Init();
  //JY901S_AccCalibrate(); // 加速度计校准
  //JY901S_MagCalibrate(); // 磁力计校准

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY,
                              ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY,
                              ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY,
                              ADC_SINGLE_ENDED);

  // ⚠️ 关键：先初始化IIR滤波器，再启动ADC DMA
  // 预处理IIR系数（将增益系数应用到SOS系数中，并修正符号）
  for(int i=0;i<IIR_STAGES;i++)
  {
	  iir_scale_value*=iir_scale_vals[i];
  }

  //iir_coeffs_preprocess();

  // 使用预处理后的系数初始化IIR滤波器
  arm_biquad_cascade_df1_init_f32(&iir_filter1, IIR_STAGES, iir_sos_coeffs,
                                  emg1_filter_state);
  arm_biquad_cascade_df1_init_f32(&iir_filter2, IIR_STAGES, iir_sos_coeffs,
                                  emg2_filter_state);

  // 初始化陷波滤波器（使用常量系数）
  arm_biquad_cascade_df1_init_f32(&notch_filter1, NOTCH_STAGES, notch_coeffs,
                                  emg1_notch_state);
  arm_biquad_cascade_df1_init_f32(&notch_filter2, NOTCH_STAGES, notch_coeffs,
                                  emg2_notch_state);

  // 生成汉明窗系数（使用自定义函数）
  hamming_window_init();

  HAL_Delay(3000);
  // 启动ADC DMA采样（滤波器已初始化完成）
  HAL_ADC_Start_DMA(&hadc3, adc3_buf, FFT_POINT);
  HAL_Delay(10);
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_dual_buf, FFT_POINT);
  HAL_Delay(10);

  // 启动定时器触发ADC采样
    HAL_TIM_Base_Start(&htim8);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    //JY901S_GetAttitude(&roll, &pitch, &yaw);         // 获取姿态角
    //JY901S_GetAccelerometer(&acc_x, &acc_y, &acc_z); // 获取加速度
    //JY901S_GetGyroscope(&gyro_x, &gyro_y, &gyro_z);  // 获取角速度
    //sprintf(message,"%.2f,%.2f,%.2f\n",roll,pitch,yaw);
    //HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
    //HAL_UART_Transmit(&huart2, (uint8_t*)"1", sizeof("1"),100);
    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 10;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
