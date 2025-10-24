FROM python:3.11-slim      # base OS + Python runtime for ARM on Pi
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
WORKDIR /app               # set working directory inside the image
COPY main.py .              # copy your code into the image
CMD ["python", "main.py"]   # default process when container starts