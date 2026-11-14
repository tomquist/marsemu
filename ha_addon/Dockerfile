ARG BUILD_FROM=python:3.13-slim
FROM ${BUILD_FROM}

# marsemu from its repository; MARSEMU_REF is a branch or tag
ARG MARSEMU_REF=main
RUN pip install --no-cache-dir "marsemu @ https://github.com/tomquist/marsemu/archive/${MARSEMU_REF}.tar.gz"

CMD ["python", "-m", "marsemu.ui.addon"]
