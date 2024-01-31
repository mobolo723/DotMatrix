#include "utility.h"
#include "font5x7.h"
#include "objects.h"

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
        // Cette ligne contrôle l'affichage du
        /*
        // bitset->matrice[pos_y + y][pos_x + x].red = 0;
        */
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

void decalage_ligne(HUB75_bitset *bitset, unsigned int ligne, int offset) {
  // offset compris dans -/+ LARGEUR_MATRICE
  offset %= LARGEUR_MATRICE;

  // Prise en compte d'un offset nul
  if (offset == 0) {
    return;
  }

  // Prise en compte des offsets négatifs
  if (offset < 0) {
    offset = LARGEUR_MATRICE + offset;
  }

  // Création d'une ligne temporaire de taille offset
  Pixel temp_ligne[offset];

  // Copie des pixels vers le tableau temporaire
  for (int i = 0; i < offset; i++) {
    temp_ligne[i] = bitset->matrice[ligne][i];
  }

  // Décalage des pixels
  for (int i = 0; i < LARGEUR_MATRICE - offset; i++) {
    bitset->matrice[ligne][i] = bitset->matrice[ligne][i + offset];
  }

  // Recopie des pixels temporaires dans la ligne
  for (int i = LARGEUR_MATRICE - offset, j = 0; i < LARGEUR_MATRICE; i++, j++) {
    bitset->matrice[ligne][i] = temp_ligne[j];
  }
}

void decalage_colonne(HUB75_bitset *bitset, unsigned int colonne, int offset) {
  // offset compris dans -/+ HAUTEUR_MATRICE
  offset %= HAUTEUR_MATRICE;

  // Prise en compte d'un offset nul
  if (offset == 0) {
    return;
  }

  // Prise en compte des offsets négatifs
  if (offset < 0) {
    offset = HAUTEUR_MATRICE + offset;
  }

  // Création d'une colonne temporaire de taille offset
  Pixel temp_colonne[offset];

  // Copie des pixels vers le tableau temporaire
  for (int i = 0; i < offset; i++) {
    temp_colonne[i] = bitset->matrice[i][colonne];
  }

  // Décalage des pixels
  for (int i = 0; i < HAUTEUR_MATRICE - offset; i++) {
    bitset->matrice[i][colonne] = bitset->matrice[i + offset][colonne];
  }

  // Recopie des pixels temporaires dans la ligne
  for (int i = HAUTEUR_MATRICE - offset, j = 0; i < HAUTEUR_MATRICE; i++, j++) {
    bitset->matrice[i][colonne] = temp_colonne[j];
  }
}

void rectangle_plein(HUB75_bitset *bitset, int x_min, int y_min, int x_max,
                     int y_max, unsigned char red, unsigned char green,
                     unsigned char blue) {
  for (int ligne = x_min; ligne <= x_max; ligne++) {
    for (int colonne = y_min; colonne <= y_max; colonne++) {
      if (ligne >= x_min || ligne <= x_max || colonne >= y_min ||
          colonne <= y_max) {
        bitset->matrice[ligne][colonne].red = red;
        bitset->matrice[ligne][colonne].green = green;
        bitset->matrice[ligne][colonne].blue = blue;
      }
    }
  }
}

void rectangle(HUB75_bitset *bitset, int x_min, int y_min, int x_max, int y_max,
               unsigned char red, unsigned char green, unsigned char blue) {
  for (int ligne = x_min; ligne <= x_max; ligne++) {
    for (int colonne = y_min; colonne <= y_max; colonne++) {
      if (ligne == x_min || ligne == x_max || colonne == y_min ||
          colonne == y_max) {
        bitset->matrice[ligne][colonne].red = red;
        bitset->matrice[ligne][colonne].green = green;
        bitset->matrice[ligne][colonne].blue = blue;
      }
    }
  }
}