# ----- Use Miniconda base image
FROM continuumio/miniconda3

# ----- Variable for the conda environment name
ARG CONDA_ENV_NAME=spacesim
ENV CONDA_DEFAULT_ENV=${CONDA_ENV_NAME}
ENV PATH=/opt/conda/envs/${CONDA_ENV_NAME}/bin:$PATH

# ----- Variable for the conda environment definition file
ARG CONDA_ENV_FILE=environment.yml

# ----- Set the working directory
WORKDIR /app

# ----- Copy the environment file into the image
COPY ${CONDA_ENV_FILE} .

# ----- Create the Conda environment
RUN conda env create -f ${CONDA_ENV_FILE}

# ----- Copy the rest of the code ---
COPY . .

# Make RUN commands use the new environment:
# SHELL ["conda", "run", "-n", "${CONDA_ENV_NAME}", "/bin/bash", "-c"]

# ----- Copy the rest of the code
COPY . .

# Set the default command to run in the Conda environment
# Note: SHELL directive above ensures this runs in the activated env
# CMD ["python", "main.py"]

# ----- Run the application
CMD ["python", "-m", "main"]

