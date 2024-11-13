from fastapi import FastAPI
import numpy as np
from kalman_filter import KalmanFilter
import time
from pydantic import BaseModel
from typing import List
import json

app = FastAPI()

# Definir el modelo para la matriz
class Matrix(BaseModel):
    matrix: List[List[float]]

# Definir el modelo para el vector
class Vector(BaseModel):
    vector: List[float]

@app.post("/kalman")
async def filter(estado_inicial: Vector,
                 control: Vector,
                 medicion: Vector,
                 matriz_estado: Matrix, 
                 matriz_control: Matrix,
                 matriz_medicion: Matrix, 
                 covarianza_proceso: Matrix,
                 covarianza_medicion: Matrix,
                 covarianza_inicial: Matrix):
    start = time.time()
    
    # Parámetros de ejemplo del filtro de Kalman
    A = matriz_estado          # Matriz de estado
    B = matriz_control         # Matriz de control
    H = matriz_medicion        # Matriz de medición
    Q = covarianza_proceso     # Covarianza de proceso
    R = covarianza_medicion    # Covarianza de medición
    
    # Crear instancia del filtro de Kalman
    kf = KalmanFilter(A.matrix, B.matrix, H.matrix, 
                      Q.matrix, R.matrix, 
                      estado_inicial.vector, covarianza_inicial.matrix)

    # Ciclo de predicción y actualización
    
    # Predicción
    kf.predict(control.vector)
    afterpred = kf.get_state()

    # Actualización
    kf.update(medicion.vector)
    afteract = kf.get_state()

    end = time.time()

    var1 = end - start

    j1 = {
        "Time taken in seconds": var1,
        "Estado despues de prediccion": afterpred,
        "Estado despues de actualizacion": afteract
    }
    jj = json.dumps(j1)

    return jj