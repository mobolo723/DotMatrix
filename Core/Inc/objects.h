/**
 ******************************************************************************
 * @file           objects.h
 * @brief          Fichier d'en-tête définissant les objets et les constantes.
 ******************************************************************************
 * @attention
 *
 * Ce fichier fait partie du projet Michel.
 * @author         Antoine DANIEL
 * @date           31 janvier 2024
 * @version        V1.0.0
 ******************************************************************************
 */

#ifndef OBJECTS_H
#define OBJECTS_H

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/**
 * @defgroup PD Private Defines
 * @{
 */
#define LARGEUR_MATRICE 128
#define HAUTEUR_MATRICE 32
/** @} */
/* USER CODE END PD */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @defgroup PTD Private Typedefs
 * @{
 */

/**
 * @brief Structure représentant un pixel avec des composantes rouge, vert et
 * bleu.
 */
typedef struct {
  unsigned char red;   /**< Intensité du rouge (0 à 255). */
  unsigned char green; /**< Intensité du vert (0 à 255). */
  unsigned char blue;  /**< Intensité du bleu (0 à 255). */
} Pixel;

/**
 * @brief Structure représentant une matrice de pixel.
 */
typedef struct {
  Pixel matrice[HAUTEUR_MATRICE][LARGEUR_MATRICE]; /**< Tableau 2D de pixels. */
} HUB75_bitset;
/** @} */
/* USER CODE END PTD */

#endif /* OBJECTS_H */
