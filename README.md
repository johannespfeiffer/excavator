# Baggerschaufelhöhe
Diese App ist für einen Baggerführer. Sie zeigt die aktuelle Höhe der Schaufelspitze, relativ zum GPS-Sensor auf dem Dach.

# Aufbau
* Auf dem Baggerdach ist ein Beschleunigungssensor S1 um die Neigung des Baggers zu erkennen, sowie ein GPS-Sensor. Länge Sensorposition bis Gelenk: L1
* Auf dem Ausleger ist ein weiterer Beschleunigungssensor S2.  Länge Ausleger ist: L2
* Auf dem Löffenstiel ist ein weiterer Beschleunigungssensor S3. Länge Löffelstiel ist: L3
* Auf dem Löffel ist ein weiterer Beschleunigungssensor S4. Länge Löffel ist L4

Die Z-Axen Beschleunigungssensoren sind orthogonal zu jeweiligen Hebelarm.

Ziel ist, durch die Rotation der Sensoren die Höhe der Schaufelspitze zu bestimmen.

# Hardware
Controller: STM-32 F446RE
Beschleunigungssensor (4 Stück): BMI160
GPS-Sensor: 
