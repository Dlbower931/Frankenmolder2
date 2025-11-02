# Use a slim Python base image
FROM python:3.11-slim

# Set the working directory inside the container
WORKDIR /app

# Copy the Python script into the container
# This assumes pi_can_handler.py is in the same directory
COPY pi_can_handler.py .

# Install python-can
# We add --no-cache-dir to reduce image size
RUN pip install --no-cache-dir python-can

# Command to run the script when the container starts
# The "-u" flag ensures Python output is unbuffered and prints to the console in real-time
CMD ["python", "-u", "./pi_can_handler.py"]

