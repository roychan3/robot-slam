FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONPATH=/app

WORKDIR /app

# Dependencies are copied and installed before the source, so editing code does
# not invalidate the install layer and rebuild every wheel.
COPY requirements.txt requirements.txt
RUN python -m pip install --upgrade pip \
    && python -m pip install -r requirements.txt

# Nothing beyond the base image is needed while requirements.txt is empty. The
# common SLAM packages do need system libraries — opencv-python wants libgl1
# and libglib2.0-0, Open3D wants libgomp1 — so add an apt-get layer above this
# one when the first of them is pinned, rather than debugging an ImportError
# that names a Python module but is really a missing .so.

COPY . .

RUN useradd --create-home --uid 10001 appuser \
    && chown -R appuser:appuser /app

USER appuser

# No entry point yet: there is nothing to run. A shell keeps the image
# inspectable, and `docker run <image> python -m <module>` already works.
CMD ["bash"]
