FROM python:3.11-slim

# Install Tkinter (not included in slim)
RUN apt-get update && apt-get install -y python3-tk && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY main.py .

# Default command
CMD ["python", "main.py"]