# Baggerschaufelhöhe
Diese App ist für einen Baggerführer. Sie zeigt die aktuelle Höhe der Schaufelspitze, relativ zum GPS-Sensor auf dem Dach.

Betriebsannahme: optimal auch während Bewegung, notfalls quasistatisch

Genauigkeitsziel: Update-Rate von 2Hz, Genauigkeitsziel von 2cm, Latenz von 10ms
Nullstellung ist alle Axen waagerecht = Höhedifferenz von Null.
Ausgabe: UART

# Anforderungen
* Zielgröße ist die Höhe der tatsächlichen Schaufelspitze.
* Die ausgegebene Höhe bemisst sich nach der seriell eingelesenen Höheninformation des GPS-Sensors auf dem Dach plus der berechneten Höhendifferenz zwischen GPS-Sensor und Schaufelspitze.
* Die Längen `L1`, `L2`, `L3`, `L4` werden als Konstanten in der Firmware hinterlegt.
* `L1` ist der Abstand von der Sensor-/GPS-Position auf dem Dach bis zum ersten Gelenk.
* `L2` ist die Länge des Auslegers.
* `L3` ist die Länge des Löffelstiels.
* `L4` ist die Länge des Löffels bis zur mechanischen Referenz am Löffel.
* Die tatsächliche Schaufelspitze ist gegenüber dieser Referenz um den konstanten Offset `O1` versetzt.
* Alle Sensoren liefern absolute Winkel gegen die Gravitation. Interessant ist nur die Höhendifferenz; Gelenkwinkel müssen nicht direkt gemessen werden.
* Für Betrieb während Bewegung soll die Lageschätzung Accelerometer und Gyroskop des BMI160 verwenden. Im Grenzfall muss die Berechnung auch quasistatisch funktionieren.
* Die Nullstellung ist erreicht, wenn alle Achsen waagerecht sind; in diesem Zustand ist die Höhendifferenz zwischen GPS-Sensor und Schaufelspitze gleich Null.

# Stack
Entwicklung in C
CMake
CMSIS
Developmentboard: Nucleo-64

# Aufbau
* Auf dem Baggerdach ist ein Beschleunigungssensor `S1` sowie Gyro `G1` (`I2C1`) zur Erkennung der Neigung des Baggers, sowie ein GPS-Sensor. Länge Sensorposition bis Gelenk: `L1`
* Auf dem Ausleger ist ein weiterer Beschleunigungssensor `S2` sowie Gyro `G2` (`I2C2`). Länge Ausleger: `L2`
* Auf dem Löffelstiel ist ein weiterer Beschleunigungssensor `S3` sowie Gyro `G3` (`I2C3`). Länge Löffelstiel: `L3`
* Auf dem Löffel ist ein weiterer Beschleunigungssensor `S4` sowie Gyro `G4` (`FMPI2C1`). Länge Löffel: `L4`

Die Z-Axen Beschleunigungssensoren sind orthogonal zu jeweiligen Hebelarm.

Ziel ist, durch die Rotation der Sensoren die Höhe der Schaufelspitze zu bestimmen.

# Hardware
Controller: STM-32 F446RE
Beschleunigungssensor + Gyro (4 Stück): BMI160 angebunden über I2C
GPS-Sensor: Höheninformation kommt über serielles Interface und dient als Referenz für die absolute Höhe der Schaufelspitze

# Implementierungsplan
1. Anforderungen und Mathematik fixieren
   * Koordinatensystem, Vorzeichenkonventionen und Formel für die Höhendifferenz festlegen.
   * `L1`, `L2`, `L3`, `L4` und `O1` als Firmware-Konstanten vorsehen.
2. Projektgerüst aufsetzen
   * `CMake`-Projektstruktur für `app`, `platform`, `drivers`, `math`, `config` und `tests` anlegen.
   * Toolchain, Startup-Code, Linker-Script sowie Targets für Build, Flash, Debug und Size vorbereiten.
3. Plattformschicht implementieren
   * Initialisierung für Takt, GPIO, UART, `I2C1`, `I2C2`, `I2C3` und `FMPI2C1` bereitstellen.
   * Minimale Self-Tests und Fehlerbehandlung ergänzen.
4. BMI160-Treiber implementieren
   * Initialisierung, Konfiguration, Chip-ID-Prüfung und Lesen von Beschleunigungs- und Gyrodaten umsetzen.
   * Gemeinsame Treiberschnittstelle für alle vier Sensoren bereitstellen.
5. GPS-Anbindung implementieren
   * UART-Empfang mit Ringpuffer und Parser für die Höheninformation umsetzen.
   * Plausibilitätsprüfung und Fehlerstatus für ungültige GPS-Daten ergänzen.
6. Datenmodell und Konfiguration definieren
   * Strukturen für Rohdaten, gefilterte Lage, Geometriekonstanten, GPS-Referenzhöhe und Ergebniswerte einführen.
7. Lageschätzung pro Sensor entwickeln
   * Zuerst eine quasistatische Winkelschätzung aus dem Beschleunigungssensor implementieren.
   * Danach Sensorfusion aus Accelerometer und Gyroskop für Betrieb während Bewegung ergänzen.
8. Kinematik implementieren
   * Aus absoluten Winkeln und den Konstanten `L1` bis `L4` sowie `O1` die Höhendifferenz zwischen GPS-Sensor und Schaufelspitze berechnen.
   * Die Kinematik als reine, hardwarefreie Mathematikfunktion kapseln.
9. Ausgabe und Diagnose ergänzen
   * UART-Ausgabe für GPS-Höhe, berechnete Höhendifferenz, absolute Schaufelhöhe und Statusinformationen bereitstellen.
10. Nullstellung und Kalibrierung umsetzen
   * Offsets für die definierte Nullstellung erfassen und in der Berechnung berücksichtigen.
11. Tests auf Host und Target ergänzen
   * Parser, Kinematik, Winkelverarbeitung und Plausibilitätsprüfungen ohne Hardware testen.
   * Integration zuerst mit einem Sensor, dann mit allen vier Sensoren und zuletzt mit GPS durchführen.
12. Validierung gegen die Zielwerte
   * Verhalten im Stillstand und während Bewegung gegen Update-Rate, Latenz und Genauigkeit prüfen.
