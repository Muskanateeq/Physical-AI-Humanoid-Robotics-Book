FROM python:3.11-slim

WORKDIR /app

# Install system dependencies. Debian names the PostgreSQL development package
# libpq-dev (not postgresql-dev).
RUN apt-get update && apt-get install -y \
    gcc \
    libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Create non-root user
RUN useradd --create-home --shell /bin/bash app \
    && chown -R app:app /app
USER app

# Render provides PORT at runtime; 10000 is its default.
EXPOSE 10000

# Use the platform-provided port, with a local-development fallback.
CMD uvicorn src.main:app --host 0.0.0.0 --port ${PORT:-10000}
