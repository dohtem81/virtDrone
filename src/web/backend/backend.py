# backend.py - FastAPI server for web interface
from fastapi import FastAPI

app = FastAPI()

@app.get("/")
def read_root():
    return {"message": "virtDrone web interface"}

# Placeholder for control endpoints, visualization data, etc.