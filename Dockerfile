FROM python:3.13-slim

WORKDIR /src
COPY pyproject.toml README.md ./
COPY marsemu ./marsemu
RUN pip install --no-cache-dir . && rm -rf /src

# firmware images are not part of the image: mount them here
WORKDIR /firmware
# /power and /status of the whole-device commands
EXPOSE 8090
ENTRYPOINT ["marsemu"]
CMD ["--help"]
