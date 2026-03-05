FROM python:3.10-slim

WORKDIR /app

# Εγκατάσταση απαραίτητων συστημικών βιβλιοθηκών για το spatial data (osmnx/geopandas)
RUN apt-get update && apt-get install -y spatialite-bin libsqlite3-mod-spatialite && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000", "--reload"]
# CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000", "--reload", "--reload-dir", "src"]
CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]