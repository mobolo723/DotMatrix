/**
 ******************************************************************************
 * @file           screen.h
 * @brief          Fichier d'en-tête définissant les fonctions d'affichage.
 ******************************************************************************
 * @attention
 *
 * Ce fichier fait partie du projet Michel
 * @author         Antoine DANIEL
 * @date           31 janvier 2024
 * @version        V1.0.0
 ******************************************************************************
 */

#ifndef SCREEN_H
#define SCREEN_H

#include "objects.h"
#include <stdint.h>

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/**
 * @defgroup PFP Private function prototypes
 * @{
 */

/**
 * @brief Initialise un tableau de pixels noirs. Toutes les composantes de
 * chaque pixel sont à zéro.
 *
 * @param bitset Pointeur vers l'ensemble de pixels de la dalle.
 */
void init_affichage(HUB75_bitset *bitset);

/**
 * @brief Écrit un pixel dans l'ensemble de pixels aux coordonnées données.
 *
 * @param bitset Pointeur vers l'ensemble de pixels de la dalle.
 * @param pox_x  Ligne du pixel.
 * @param pos_y  Colonne du pixel.
 * @param pixel  Pixel à écrire.
 */
void ecrire_pixel(HUB75_bitset *bitset, int pos_x, int pos_y, Pixel pixel);

/**
 * @brief Éteint un pixel dans l'ensemble de pixels aux coordonnées données.
 *
 * @param bitset Pointeur vers l'ensemble de pixels de la dalle.
 * @param pox_x  Ligne du pixel.
 * @param pos_y  Colonne du pixel.
 */
void eteindre_pixel(HUB75_bitset *bitset, int pos_x, int pos_y);

/**
 * @brief Crée une attente virtuelle dans le programme.
 *
 * @note Les mesures d'attentes ont été réalisées sur NUCLEO L452RE.
 *
 * @param n Temps d'attente. 3920720 correspond à 1 seconde. 1 correspond à 680
 * nanosecondes.
 */
void delai(uint32_t n);

/**
 * @brief Éteint un pixel dans l'ensemble de pixels aux coordonnées données.
 *
 * @param red   Intensité de la couleur rouge.
 * @param green Intensité de la couleur verte.
 * @param blue  Intensité de la couleur bleue.
 *
 * @return Élément de type Pixel.
 */
Pixel creer_pixel(unsigned char red, unsigned char green, unsigned char blue);

/** @} */
/* USER CODE END PFP */
#endif