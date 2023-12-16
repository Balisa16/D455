FROM gcc:9.4

COPY . /usr/src/d455_app
WORKDIR /usr/src/d455_app

# Prebuild
RUN apt-get update && \
    apt-get install -y libboost-all-dev && \
    apt-get install -y libpcl-dev python3-pcl && \
    rm -rf /var/lib/apt/lists/*
RUN git submodule update --init
RUN ./prebuild.sh

# Build
RUN rm -rf build && mkdir build
WORKDIR /usr/src/d455_app/build
RUN cmake ..
RUN make

# Run Unit Test
WORKDIR /usr/src/d455_app/release/unit_test
CMD ["./mat_test"]