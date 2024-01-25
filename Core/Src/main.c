/* USER CODE BEGIN Header */

/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "font5x7.h"
#include <stdio.h>
#include <string.h>

#define LARGEUR_MATRICE 128
#define HAUTEUR_MATRICE 32
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  // Intensité de la couleur entre 0 et 255
  unsigned char red, green, blue;
} Pixel;

typedef struct {
  Pixel matrice[HAUTEUR_MATRICE][LARGEUR_MATRICE];
} HUB75_bitset;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const Pixel RED_Pixel = {255, 0, 0};
const Pixel GREEN_Pixel = {0, 255, 0};
const Pixel BLUE_Pixel = {0, 0, 255};
const Pixel WHITE_Pixel = {255, 255, 255};
const Pixel BLACK_Pixel = {0, 0, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
/**
 * @brief Affiche l'ensemble de pixels sur la dalle. Correspond à un
 * rafraîchissement.
 *
 * @param bitset Pointeurs vers l'ensemble de pixels de la dalle.
 */
void Affichage(HUB75_bitset *bitset);

HUB75_bitset init_bitset();
Pixel creer_pixel(unsigned char red, unsigned char green, unsigned char blue);
unsigned char ReadBCD();
void affichage_bitset(HUB75_bitset *bitset);
void affiche_lettre(HUB75_bitset *bitset, unsigned int pos_x,
                    unsigned int pos_y, unsigned char red, unsigned char green,
                    unsigned char blue, unsigned char lettre);
void affiche_phrase(HUB75_bitset *bitset, unsigned int pos_x,
                    unsigned int pos_y, unsigned char red, unsigned char green,
                    unsigned char blue, unsigned char *phrase);
void affiche_rectangle(HUB75_bitset *bitset, unsigned int borne_min_x,
                       unsigned int borne_max_x, unsigned int borne_min_y,
                       unsigned int borne_max_y, Pixel pixel);
void decaler_ligne(HUB75_bitset *bitset, unsigned int ligne);
void delai(uint16_t n);
void ecrire_pixel(HUB75_bitset *bitset, int pos_x, int pos_y, Pixel pixel);
void eteindre_pixel(HUB75_bitset *bitset, int pos_x, int pos_y);
void ReadScore(char *score);
void ResetDisplay(HUB75_bitset *bitset);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Initialisation de toutes les broches à l'état par défaut
  // Output Enable (par défaut à : Bas)
  HAL_GPIO_WritePin(LAT_GPIO_Port, LAT_Pin, RESET);

  // Output Enable (par défaut à : Haut)
  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);

  // Output Enable (par défaut à : Bas)
  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, RESET);
  HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, RESET);
  HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, RESET);
  HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, RESET);
  HAL_GPIO_WritePin(G1_GPIO_Port, G1_Pin, RESET);
  HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, RESET);

  // Initialisation de la matrice de pixels
  HUB75_bitset affichage = init_bitset();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim16);

  for (int i = 0; i < 32; i++) {
	  for (int j = 0; j < 128 / 3; j++) {
		  ecrire_pixel(&affichage, i, (j) % 128, BLUE_Pixel);
		  ecrire_pixel(&affichage, i, (j + 128/3) % 128, WHITE_Pixel);
		  ecrire_pixel(&affichage, i, (j + 2*128/3) % 128, RED_Pixel);
	  }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  Affichage(&affichage);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, G2_Pin|R2_Pin|G1_Pin|R1_Pin
                          |B_Pin|D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TEST_Pin|B1_Pin|CLK_Pin|OE_Pin
                          |LAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, C_Pin|A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : G2_Pin R2_Pin G1_Pin R1_Pin
                           B_Pin D_Pin */
  GPIO_InitStruct.Pin = G2_Pin|R2_Pin|G1_Pin|R1_Pin
                          |B_Pin|D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TEST_Pin B1_Pin CLK_Pin OE_Pin
                           LAT_Pin */
  GPIO_InitStruct.Pin = TEST_Pin|B1_Pin|CLK_Pin|OE_Pin
                          |LAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C_Pin A_Pin */
  GPIO_InitStruct.Pin = C_Pin|A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STROBE_Pin D2_Pin D3_Pin D4_Pin
                           D5_Pin D6_Pin BLANKING_Pin D1_Pin */
  GPIO_InitStruct.Pin = STROBE_Pin|D2_Pin|D3_Pin|D4_Pin
                          |D5_Pin|D6_Pin|BLANKING_Pin|D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BCD3_Pin */
  GPIO_InitStruct.Pin = BCD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BCD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BCD2_Pin BCD1_Pin BCD0_Pin */
  GPIO_InitStruct.Pin = BCD2_Pin|BCD1_Pin|BCD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B2_Pin */
  GPIO_InitStruct.Pin = B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(B2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
HUB75_bitset init_bitset() {
  HUB75_bitset bitset;
  for (int i = 0; i < HAUTEUR_MATRICE; i++) {
    for (int j = 0; j < LARGEUR_MATRICE; j++) {
      bitset.matrice[i][j].red = 0;
      bitset.matrice[i][j].green = 0;
      bitset.matrice[i][j].blue = 0;
    }
  }

  return bitset;
}

void Affichage(HUB75_bitset *bitset) {
	uint16_t t = 50;
  /* Sélection des lignes */
  for (unsigned int ligne = 0; ligne < HAUTEUR_MATRICE / 2; ligne++) {
    if (ligne & 0b0001) {
      C_GPIO_Port->BSRR = (uint32_t)C_Pin;
    } else {
      C_GPIO_Port->BRR = (uint32_t)C_Pin;
    }

    if (ligne & 0b0010) {
      D_GPIO_Port->BSRR = (uint32_t)D_Pin;
    } else {
      D_GPIO_Port->BRR = (uint32_t)D_Pin;
    }

    if (ligne & 0b0100) {
      A_GPIO_Port->BSRR = (uint32_t)A_Pin;
    } else {
      A_GPIO_Port->BRR = (uint32_t)A_Pin;
    }

    if (ligne & 0b1000) {
      B_GPIO_Port->BSRR = (uint32_t)B_Pin;
    } else {
      B_GPIO_Port->BRR = (uint32_t)B_Pin;
    }

    /* Sélection des colonnes */
    for (unsigned int colonne = 0; colonne < LARGEUR_MATRICE; colonne++) {
      /* Rouge R1 */
      if (bitset->matrice[ligne][colonne].red & 0xff) {
        R1_GPIO_Port->BSRR = (uint32_t)R1_Pin;
      }

      /* Rouge R2 */
      if (bitset->matrice[ligne + 16][colonne].red & 0xff) {
        R2_GPIO_Port->BSRR = (uint32_t)R2_Pin;
      }

      /* Vert G1 */
      if (bitset->matrice[ligne][colonne].green & 0xff) {
        G1_GPIO_Port->BSRR = (uint32_t)G1_Pin;
      }

      /* Vert G2 */
      if (bitset->matrice[ligne + 16][colonne].green & 0xff) {
        G2_GPIO_Port->BSRR = (uint32_t)G2_Pin;
      }

      /* Bleu B1 */
      if (bitset->matrice[ligne][colonne].blue & 0xff) {
        B1_GPIO_Port->BSRR = (uint32_t)B1_Pin;
      }

      /* Bleu B2 */
      if (bitset->matrice[ligne + 16][colonne].blue & 0xff) {
        B2_GPIO_Port->BSRR = (uint32_t)B2_Pin;
      }

      /* Horloge */
      CLK_GPIO_Port->BSRR = (uint32_t)CLK_Pin;
      CLK_GPIO_Port->BRR = (uint32_t)CLK_Pin;

      /* RESET de toutes les broches de couleurs */
      R1_GPIO_Port->BRR = (uint32_t)R1_Pin;
      R2_GPIO_Port->BRR = (uint32_t)R2_Pin;
      B1_GPIO_Port->BRR = (uint32_t)B1_Pin;
      B2_GPIO_Port->BRR = (uint32_t)B2_Pin;
      G1_GPIO_Port->BRR = (uint32_t)G1_Pin;
      G2_GPIO_Port->BRR = (uint32_t)G2_Pin;
    }

    /* Latch */
    LAT_GPIO_Port->BSRR = (uint32_t)LAT_Pin;
    LAT_GPIO_Port->BRR = (uint32_t)LAT_Pin;

    /* Output Enabled */
    OE_GPIO_Port->BRR = (uint32_t)OE_Pin;
    delai(500);
    OE_GPIO_Port->BSRR = (uint32_t)OE_Pin;
  }
}

void affiche_lettre(HUB75_bitset *bitset, unsigned int pos_x,
                    unsigned int pos_y, unsigned char red, unsigned char green,
                    unsigned char blue, unsigned char lettre) {
  for (unsigned int x = 0; x < 5; x++) {
    for (unsigned int y = 0; y < 7; y++) {
      if (font5x7[lettre * 5 + x] >> y & 1) {
        bitset->matrice[pos_y + y][pos_x + x].red = red;
        bitset->matrice[pos_y + y][pos_x + x].blue = blue;
        bitset->matrice[pos_y + y][pos_x + x].green = green;
      } else {
        // bitset->matrice[pos_y + y][pos_x + x].red = 0;
      }
    }
  }
}

void affiche_phrase(HUB75_bitset *bitset, unsigned int pos_x,
                    unsigned int pos_y, unsigned char red, unsigned char green,
                    unsigned char blue, unsigned char *phrase) {
  for (int index = 0; phrase[index] != '\0'; index++) {

    affiche_lettre(bitset, pos_x, pos_y, red, green, blue, phrase[index]);
    pos_x += 5;
  }
}

void ecrire_pixel(HUB75_bitset *bitset, int pos_x, int pos_y, Pixel pixel) {
  bitset->matrice[pos_x][pos_y] = pixel;
}

void eteindre_pixel(HUB75_bitset *bitset, int pos_x, int pos_y) {
  ecrire_pixel(bitset, pos_x, pos_y, creer_pixel(0, 0, 0));
}

void affiche_rectangle(HUB75_bitset *bitset, unsigned int borne_min_x,
                       unsigned int borne_max_x, unsigned int borne_min_y,
                       unsigned int borne_max_y, Pixel pixel) {
  for (unsigned int i = borne_min_x; i <= borne_max_x; i++) {
    for (unsigned int j = borne_min_y; j <= borne_max_y; j++) {
      if (j >= borne_min_y || j <= borne_max_y || i >= borne_min_x ||
          i <= borne_max_x) {
        ecrire_pixel(bitset, i, j, pixel);
      }
    }
  }
}

/**
 * @brief Décale les pixels d'une ligne de la matrice d'un vers la gauche.
 * @param bitset Pointteur vers le bitset HUB75
 * @param ligne Ligne à décaler
 */
void decaler_ligne(HUB75_bitset *bitset, unsigned int ligne) {
  // offset %= LARGEUR_MATRICE;
  Pixel temp_pixel;
  temp_pixel = bitset->matrice[ligne][0];
  for (int i = 0; i < LARGEUR_MATRICE; i++) {
    bitset->matrice[ligne][i] = bitset->matrice[ligne][i + 1];
  }
  bitset->matrice[ligne][LARGEUR_MATRICE - 1] = temp_pixel;
}

Pixel creer_pixel(unsigned char red, unsigned char green, unsigned char blue) {
  Pixel pixel;
  pixel.red = red;
  pixel.green = green;
  pixel.blue = blue;
  return pixel;
}

void affichage_bitset(HUB75_bitset *bitset) {
  // L'utilisation de pointeurs a été préférée à celle de variables car ils
  // peuvent être modifiés par la fonction tout en gardant la structure de
  // données initiale. Cela permet également d'avoir de meilleures perfomances
  // et de ne pas avoir à refaire une copie de la structure à chaque fois
  // qu'on appelle la fonction, la structure pouvant être très lourde

  // Idée d'implémentation pour la luminosité : Crée un masque BCM avec un
  // timer lors de l'écriture de la couleur sur le registre. Exemple : La
  // couleur rouge est à 165 (0b10100101), on fait varier un timer à 8 bits
  // sur un intervale de temps maximum (40 ms pour 25 ips donc 2.5 ms pour 32
  // lignes) et grâce à un masque on décide de l'écriture du bit ou non. Au
  // lieu d'avoir un masque | 0xff

  HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, SET);
  for (int i = 0; i < HAUTEUR_MATRICE / 2;
       i++) { // Implémenter le scan line ici
    // Sélection des lignes
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, i & 0b0001); // 1 ligne
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, i & 0b0010); // 2 ligne
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, i & 0b0100); // 4 ligne
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, i & 0b1000); // 8 ligne
    for (int j = 0; j < LARGEUR_MATRICE; j++) {
      // Sélection des colonnes
      // Pixel rouge
      HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, bitset->matrice[i][j].red & 0xff);
      HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin,
                        bitset->matrice[i + 16][j].red & 0xff);

      // Pixel vert
      HAL_GPIO_WritePin(G1_GPIO_Port, G1_Pin,
                        bitset->matrice[i][j].green & 0xff);
      HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin,
                        bitset->matrice[i + 16][j].green & 0xff);

      // Pixel bleu
      HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin,
                        bitset->matrice[i][j].blue & 0xff);
      HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin,
                        bitset->matrice[i + 16][j].blue & 0xff);

      // Activation de l'horloge pour décaler les bits du registre
      HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, SET);
      HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, RESET);
    }

    // Latch
    HAL_GPIO_WritePin(LAT_GPIO_Port, LAT_Pin, SET);
    HAL_GPIO_WritePin(LAT_GPIO_Port, LAT_Pin, RESET);

    // Output Enabled
    HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, RESET);
    delai(500);
    HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
  }
  HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, RESET);
}

unsigned char ReadBCD() {
  unsigned char BCD;
  BCD = HAL_GPIO_ReadPin(BCD3_GPIO_Port, BCD3_Pin);
  BCD = (BCD << 1) | HAL_GPIO_ReadPin(BCD2_GPIO_Port, BCD2_Pin);
  BCD = (BCD << 1) | HAL_GPIO_ReadPin(BCD1_GPIO_Port, BCD1_Pin);
  BCD = (BCD << 1) | HAL_GPIO_ReadPin(BCD0_GPIO_Port, BCD0_Pin);
  return BCD;
}

void ResetDisplay(HUB75_bitset *bitset) {
  for (unsigned int i=0; i<HAUTEUR_MATRICE; i++) {
    for (unsigned int j=0; j<LARGEUR_MATRICE; j++) {
      bitset->matrice[i][j].red = 0;
      bitset->matrice[i][j].green = 0;
      bitset->matrice[i][j].blue = 0;
    }
  }
}

void ReadScore(char *score) {
  unsigned char digit = 0, digit_score = ReadBCD();

  digit += HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin) * 5;
  digit += HAL_GPIO_ReadPin(D2_GPIO_Port, D2_Pin) * 4;
  digit += HAL_GPIO_ReadPin(D3_GPIO_Port, D3_Pin) * 3;
  digit += HAL_GPIO_ReadPin(D4_GPIO_Port, D4_Pin) * 2;
  digit += HAL_GPIO_ReadPin(D5_GPIO_Port, D5_Pin) * 1;
  digit += HAL_GPIO_ReadPin(D6_GPIO_Port, D6_Pin) * 0;

  if (digit_score > 9) {
    score[digit] = ' ';
    return;
  }
  switch (digit) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
      score[digit] = '0' + digit_score;
      break;

    default:
      score[digit] = ' ';
      break;
  }
}

void delai(uint16_t n) {
  for (uint16_t i = 0; i < n; i++) {
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
