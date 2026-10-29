# Anémomètre connecté (ESP32-C3 + BLE)

Anémomètre à coupelles qui mesure la vitesse du vent avec un capteur à effet Hall et l'envoie en Bluetooth Low Energy.

Projet réalisé dans le cours Projet Environnement II au Collège de Maisonneuve, avec Juliette Beaulieu et Karim Atié.

## Fonctionnement

- Deux aimants sont fixés sur le rotor. À chaque passage devant le capteur Hall, l'ESP32 mesure le temps écoulé depuis le passage précédent et en déduit la vitesse.
- Un antirebond de 40 ms ignore les fausses détections.
- Les mesures sont groupées par 10. Le minimum et le maximum sont retirés, puis la valeur qui a le plus de mesures proches (à ±2 km/h) est retenue.
- Sans détection pendant 3 s, la vitesse retombe à 0.
- La vitesse est envoyée chaque seconde en BLE et affichée sur le moniteur série.

## Structure

```
platformio.ini   configuration de la carte
src/main.cpp     code de l'anémomètre
```
