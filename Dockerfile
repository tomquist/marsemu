FROM python:3.13-slim

WORKDIR /src
COPY pyproject.toml setup.py README.md ./
COPY marsemu ./marsemu
# gcc only to build the optional C2000 accelerator (marsemu/c28x/_c28x.c); it goes again after
RUN apt-get update && apt-get install -y --no-install-recommends gcc libc6-dev \
    && pip install --no-cache-dir . && rm -rf /src \
    && apt-get purge -y gcc libc6-dev && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# firmware images are not part of the image: mount them here
WORKDIR /firmware
# /power and /status of the whole-device commands; the dashboard (marsemu ui)
EXPOSE 8090 8080
ENTRYPOINT ["marsemu"]
CMD ["--help"]
