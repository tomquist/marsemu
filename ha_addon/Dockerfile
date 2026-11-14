# Python on Debian (glibc), so pip finds the prebuilt unicorn and capstone
# wheels for amd64 and aarch64. The Supervisor's BUILD_FROM (an Alpine image
# without Python) is not used on purpose.
FROM python:3.13-slim

# marsemu from its repository; MARSEMU_REF is a branch, tag or commit.
# The Supervisor passes BUILD_VERSION (config.yaml's version): naming it here
# makes a new version download marsemu again instead of reusing Docker's cache.
ARG MARSEMU_REF=ccr-bdbc19dc-vmgsca
ARG BUILD_VERSION
RUN echo "add-on ${BUILD_VERSION}" && pip install --no-cache-dir "marsemu @ https://github.com/tomquist/marsemu/archive/${MARSEMU_REF}.tar.gz"

CMD ["python", "-m", "marsemu.ui.addon"]
