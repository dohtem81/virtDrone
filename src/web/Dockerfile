# Dockerfile for web component (Python/FastAPI)
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Copy requirements and install
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy web source
COPY src/web/ ./src/web/

# Expose port
EXPOSE 8000

# Run the FastAPI app
CMD ["uvicorn", "src.web.backend.backend:app", "--host", "0.0.0.0", "--port", "8000"]