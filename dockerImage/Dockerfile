# Dockerfile to support the CI test and software development

FROM rust:latest

LABEL author="Te-Wei Tsai <ttsai@lsst.org>"

ARG LSST_USER=lsst
ARG LSST_USER_ID=73006
ARG LSST_USER_HOME=/home/${LSST_USER}

USER root

# Install the dependencies
RUN rustup component add llvm-tools \
    && cargo install cargo-nextest cargo-llvm-cov

# Change the user mode
RUN chmod -R 777 /usr/local/cargo

# Add LSST user
RUN groupadd -g ${LSST_USER_ID} ${LSST_USER} \
    && useradd -ms /bin/bash ${LSST_USER} -u ${LSST_USER_ID} -g ${LSST_USER}

# Change to LSST user
USER ${LSST_USER}

# Set the default environment
SHELL ["/bin/bash", "-lc"]
WORKDIR ${LSST_USER_HOME}
