# Rilevamento delle Corsie su PiCar-X

Questo progetto implementa capacità autonome di rilevamento e seguimento delle corsie per il robot PiCar-X utilizzando tecniche di visione artificiale. Il sistema utilizza una telecamera Raspberry Pi per rilevare le linee di demarcazione delle corsie e guida automaticamente il veicolo per mantenerlo all'interno dei limiti della corsia.

## Panoramica

Il progetto consiste in due principali implementazioni:

1. `` - Rilevamento tramite Lane Width delle corsie tramite trasformazione a vista dall'alto (Bird’s Eye View)
2. `` - Rilevamento tramite Sliding Windows delle corsie con l'algoritmo delle finestre scorrevoli

Entrambe le implementazioni utilizzano OpenCV per l'elaborazione delle immagini e l’API del PiCar-X per il controllo del veicolo.

## Caratteristiche

- **Rilevamento in tempo reale delle corsie** usando il feed della telecamera
- **Controllo di sterzata autonomo** basato sulla posizione rilevata della corsia
- **Algoritmi multipli di rilevamento** per diversi scenari
- **Gestione degli errori robusta** con meccanismi di recupero
- **Feedback visivo** con output video annotato
- **Controllo della velocità** con considerazioni di sicurezza

## Requisiti Hardware

- Robot PiCar-X
- Raspberry Pi con modulo fotocamera (compatibile con Picamera2)
- Illuminazione adeguata per il rilevamento delle corsie
- Linee di corsia ben definite (bianche o gialle)

## Dipendenze Software

```bash
# Pacchetti Python richiesti
pip install opencv-python
pip install numpy
pip install picamera2
pip install picarx
```

## Dettagli dell’Implementazione

### 1. Rilevamento Base delle Corsie (`lane_width.py`)

Questa implementazione utilizza un approccio più semplice, adatto a corsie ben definite:

#### Caratteristiche Principali:

- **Trasformazione a Vista dall’Alto**: converte la vista prospettica in una vista dall’alto
- **Filtro colore HSV**: rileva le linee bianche e gialle
- **Rilevamento Linee Hough**: identifica linee di corsia dritte
- **Calcolo dinamico del centro**: si adatta al rilevamento di una o due corsie
- **Smorzamento dell’angolo**: riduce le oscillazioni dello sterzo

#### Parametri:

```python
SPEED = 1                    # Velocità di movimento (1-100)
MIN_ANGLE = -40.0           # Angolo minimo di sterzata
MAX_ANGLE = 40.0            # Angolo massimo di sterzata
ANGLE_SMOOTHING = 0.2       # Fattore di smorzamento per lo sterzo
lane_width = 280            # Larghezza attesa della corsia in pixel
```

### 2. Rilevamento Avanzato delle Corsie (`sliding_windows.py`)

Questa implementazione utilizza un approccio più sofisticato con finestre scorrevoli:

#### Caratteristiche Principali:

- **Analisi dell’istogramma**: identifica i punti di partenza delle linee di corsia
- **Ricerca con finestre scorrevoli**: traccia le linee della corsia dal basso verso l’alto
- **Analisi della curvatura**: rileva e gestisce corsie curve
- **Tracciamento basato sulla memoria**: mantiene la posizione della corsia tra i fotogrammi
- **Recupero robusto da errori**: gestisce rilevamenti mancanti o invalidi

#### Parametri:

```python
DRIVING_SPEED = 0.3         # Velocità del veicolo (0-1)
WINDOW_SEARCH_MARGIN = 50   # Margine di ricerca per le finestre
MIN_CONTOUR_AREA_IN_WINDOW = 30  # Soglia minima di area del contorno
estimated_lane_width = 619  # Larghezza attesa della corsia in pixel
```

## Utilizzo

### Rilevamento Lane Width delle Corsie

```bash
python3 lane_width.py
```

### Rilevamento Sliding Windows delle Corsie

```bash
python3 sliding_windows.py
```

### Comandi:

- **'q'**: Chiude il programma
- **'p'**: Pausa il movimento (`lane_width.py`)
- **'l'**: Riprende il movimento (`lane_width.py`)
- **ESC**: Esce (`sliding_windows.py`)

## Configurazione

### Calibrazione della Telecamera

Entrambi gli script possono richiedere la calibrazione della telecamera per prestazioni ottimali:

1. **Regolare i punti di trasformazione prospettica** in `sliding_windows.py`:

```python
tl, bl, tr, br = (70,260), (0,480), (570,260), (640,480)
```

2. **Regolare gli intervalli HSV** utilizzando i trackbar in `sliding_windows.py`

3. **Modificare le impostazioni ROI** in `lane_width.py`:

```python
roi_top, roi_bottom = int(h * 0.6), h
```

### Ottimizzazione delle Prestazioni

#### Per una Maggiore Precisione:

- Aumentare `MIN_POINTS_FOR_STABLE_LINE` per rilevamenti più stabili
- Regolare `ANGLE_SMOOTHING` per una sterzata più fluida
- Ottimizzare gli intervalli HSV in base all'illuminazione

#### Per Migliori Prestazioni:

- Ridurre la risoluzione dell’immagine
- Diminuire `num_windows` nell’algoritmo delle finestre
- Regolare `DRIVING_SPEED` per un funzionamento sicuro

## Risoluzione dei Problemi

### Problemi Comuni:

1. **Rilevamento delle corsie scadente**:

   - Controllare le condizioni di luce
   - Regolare gli intervalli HSV
   - Verificare la posizione della telecamera

2. **Sterzate erratiche**:

   - Aumentare il valore di `ANGLE_SMOOTHING`
   - Verificare la stima della larghezza della corsia
   - Controllare la trasformazione prospettica

3. **Il veicolo non si muove**:

   - Verificare il parametro `DRIVING_SPEED`
   - Controllare i collegamenti del PiCar-X
   - Assicurarsi della corretta inizializzazione