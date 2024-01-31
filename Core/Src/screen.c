#include <stdint.h>
#include "screen.h"
#include "objects.h"

void init_affichage(HUB75_bitset *bitset) {
  for (int ligne=0; ligne<HAUTEUR_MATRICE; ligne++) {
    for (int colonne=0; colonne<LARGEUR_MATRICE; colonne++) {
      bitset->matrice[ligne][colonne].red = 0;
      bitset->matrice[ligne][colonne].green = 0;
      bitset->matrice[ligne][colonne].blue = 0;
    }
  }
}

void ecrire_pixel(HUB75_bitset *bitset, int pos_x, int pos_y, Pixel pixel) {
  bitset->matrice[pos_y][pos_x] = pixel;
}

void eteindre_pixel(HUB75_bitset *bitset, int pos_x, int pos_y) {
  ecrire_pixel(bitset, pos_x, pos_y, (Pixel){0, 0, 0});
}

void delai(uint32_t n) {
  for (uint32_t i = 0; i < n; i++) {
  }
}

Pixel creer_pixel(unsigned char red, unsigned char green, unsigned char blue) {
  Pixel pixel;  
  pixel.red = red;
  pixel.green = green;
  pixel.blue = blue;
  return pixel;
}