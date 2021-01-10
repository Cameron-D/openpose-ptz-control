# https://hub.docker.com/r/cwaffles/openpose
FROM nvidia/cuda:11.1-cudnn8-devel-ubuntu18.04

#get deps
RUN apt-get update && \
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
python3-dev python3-pip git g++ wget make libprotobuf-dev protobuf-compiler libopencv-dev \
libgoogle-glog-dev libboost-all-dev libcaffe-cuda-dev libhdf5-dev libatlas-base-dev nano python3-setuptools

#for python api
RUN pip3 install --upgrade pip && pip3 install numpy opencv-python paho-mqtt

#replace cmake as old version has CUDA variable bugs
RUN wget https://github.com/Kitware/CMake/releases/download/v3.16.0/cmake-3.16.0-Linux-x86_64.tar.gz && \
tar xzf cmake-3.16.0-Linux-x86_64.tar.gz -C /opt && \
rm cmake-3.16.0-Linux-x86_64.tar.gz
ENV PATH="/opt/cmake-3.16.0-Linux-x86_64/bin:${PATH}"

#get openpose
WORKDIR /openpose
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git . 
#    git checkout v1.7.0 && \
#    sed -i 's/1807aad/b846ff93/' CMakeLists.txt

RUN cd /openpose/models && bash getModels.sh

RUN mkdir /openpose/build && \
    cd /openpose/build && cmake -DBUILD_PYTHON=ON ..

RUN cd /openpose/build && \
    make -j 12 && \
    cd /openpose/build/python/openpose && make install && \
    cp ./pyopenpose.cpython-36m-x86_64-linux-gnu.so /usr/local/lib/python3.6/dist-packages && \
    cd /usr/local/lib/python3.6/dist-packages && \
    ln -s pyopenpose.cpython-36m-x86_64-linux-gnu.so pyopenpose

ENV LD_LIBRARY_PATH=/openpose/build/python/openpose

WORKDIR /ptztrack
COPY pose.py .

EXPOSE 5000
CMD ["python3", "pose.py"]
