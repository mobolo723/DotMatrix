/**
 ******************************************************************************
 * @file           utility.h
 * @brief          Fichier d'en-tête définissant les fonctions utilitaires.
 ******************************************************************************
 * @attention
 *
 * Ce fichier fait partie du projet Michel
 * @author         Antoine DANIEL
 * @date           31 janvier 2024
 * @version        V1.0.0
 ******************************************************************************
 */

#ifndef UTILITY_H
#define UTILITY_H

#include "objects.h"

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/**
 * @defgroup PFP Private function prototypes
 * @{
 */

/**
 * @brief Affiche une lettre aux coordonnées choisies
 *
 * @param bitset    Pointeur vers l'ensemble de pixels de la dalle.
 * @param pos_x     Ligne où afficher la lettre.
 * @param pos_y     Colonne où afficher la lettre.
 * @param red       Intensité de la couleur rouge.
 * @param green     Intensité de la couleur verte.
 * @param blue      Intensité de la couleur bleue.
 * @param lettre    Lettre à afficher. Tous les caractères ASCII sont pris en
 * charge.
 *
 * @note Les caractères ASCII sont pris en charge par la fonction
 * @note Les coordonnées correspondent au coin supérieur gauche de la lettre.
 * @note La police matricielle de base est de 5x7 pixels.
 */
void affiche_lettre(HUB75_bitset *bitset, unsigned int pos_x,
                    unsigned int pos_y, unsigned char red, unsigned char green,
                    unsigned char blue, unsigned char lettre);

/**
 * @brief Affiche une phrase à partir des coordonées de départ
 *
 * @param bitset    Pointeur vers l'ensemble de pixels de la dalle.
 * @param pos_x     Ligne de départ.
 * @param pos_y     Colonne de départ.
 * @param red       Intensité de la couleur rouge.
 * @param green     Intensité de la couleur verte.
 * @param blue      Intensité de la couleur bleue.
 * @param phrase    Pointeur vers la phrase à afficher.
 *
 * @note La fonction ne gère pas les retours à la ligne pour les phrases trop
 * longues.
 * @note La position de départ correspond au coin supérieur gauche de la
 * première lettre.
 *
 * @todo Ajouter un retour à la ligne à la fin de la phrase.
 */
void affiche_phrase(HUB75_bitset *bitset, unsigned int pos_x,
                    unsigned int pos_y, unsigned char red, unsigned char green,
                    unsigned char blue, unsigned char *phrase);

/**
 * @brief Décale une ligne vers la gauche ou la droite.
 *
 * @param bitset    Pointeur vers l'ensemble de pixels de la dalle.
 * @param ligne     Ligne à décaler.
 * @param offset    Offset de décalage. Un offset positif correspond à un
 * décalage vers la droite. Un offset négatif correspond à un décalage vers la
 * gauche.
 */
void decalage_ligne(HUB75_bitset *bitset, unsigned int ligne, int offset);

/**
 * @brief Décale une colonne vers le haut ou vers le bas.
 *
 * @param bitset    Pointeur vers l'ensemble de pixels de la dalle.
 * @param ligne     Colonne à décaler.
 * @param offset    Offset de décalage. Un offset positif correspond à un
 * décalage vers le bas. Un offset négatif correspond à un décalage vers le
 * haut.
 */
void decalage_colonne(HUB75_bitset *bitset, unsigned int colonne, int offset);

/**
 * @brief Modifie un ensemble de pixels HUB75_bitset et construit un rectangle
 * plein de couleur donnée aux coordonnées voulues.
 *
 * @param bitset       Pointeur vers l'ensemble de pixels de la dalle.
 * @param x_min        Ligne du coin supérieur gauche du rectangle.
 * @param y_min        Colonne du coin supérieur gauche du rectangle.
 * @param x_max        Ligne du coin inférieur droit du rectangle.
 * @param y_max        Colonne du coin inférieur droit du rectangle.
 * @param red          Composante rouge du rectangle. (0-255)
 * @param green        Composante verte du rectangle. (0-255)
 * @param blue         Composante bleue du rectangle. (0-255)
 */
void rectangle_plein(HUB75_bitset *bitset, int x_min, int y_min, int x_max,
                     int y_max, unsigned char red, unsigned char green,
                     unsigned char blue);

/**
 * @brief Modifie un ensemble de pixels HUB75_bitset et construit un rectangle
 * de couleur donnée aux coordonnées voulues.
 *
 * @param bitset       Pointeur vers l'ensemble de pixels de la dalle.
 * @param x_min        Ligne du coin supérieur gauche du rectangle.
 * @param y_min        Colonne du coin supérieur gauche du rectangle.
 * @param x_max        Ligne du coin inférieur droit du rectangle.
 * @param y_max        Colonne du coin inférieur droit du rectangle.
 * @param red          Composante rouge du rectangle. (0-255)
 * @param green        Composante verte du rectangle. (0-255)
 * @param blue         Composante bleue du rectangle. (0-255)
 */
void rectangle(HUB75_bitset *bitset, int x_min, int y_min, int x_max, int y_max,
               unsigned char red, unsigned char green, unsigned char blue);
/** @} */
/* USER CODE END PFP */
#endif