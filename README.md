# Baggerschaufelhöhe
Diese App ist für einen Baggerführer. Sie zeigt die aktuelle Höhe der Schaufelspitze, relativ zum GPS-Sensor auf dem Dach.

Betriebsanname: optimal auch während Bewegung, notfalls quasistatisch

Genauigkeitsziel: Update-Rate von 2Hz, Genauigkeitsziel von 2cm, Latenz von 10ms
Nullstellung ist alle Axen waagerecht = Höhedifferenz von Null.
Ausgabe: UART

# Stack
Entwicklung in C
CMake
CMSIS
Developmentboard: Nucleo-64

# Aufbau
* Auf dem Baggerdach ist ein Beschleunigungssensor S1 sowie Gyro G1 (I2C1) um die Neigung des Baggers zu erkennen, sowie ein GPS-Sensor. Länge Sensorposition bis Gelenk: L1
* Auf dem Ausleger ist ein weiterer Beschleunigungssensor S2 sowie Gyro G2 (I2C2).  Länge Ausleger ist: L2
* Auf dem Löffenstiel ist ein weiterer Beschleunigungssensor S3 sowie G3 (I2C3). Länge Löffelstiel ist: L3
* Auf dem Löffel ist ein weiterer Beschleunigungssensor S4 sowie G4 (FMPI2C1). Länge Löffel ist L4

Die Z-Axen Beschleunigungssensoren sind orthogonal zu jeweiligen Hebelarm.

Ziel ist, durch die Rotation der Sensoren die Höhe der Schaufelspitze zu bestimmen.

# Hardware
Controller: STM-32 F446RE
Beschleunigungssensor + Gyro (4 Stück): BMI160 angebunden über I2C
GPS-Sensor: Höheninformation kommen über serielles Interface
