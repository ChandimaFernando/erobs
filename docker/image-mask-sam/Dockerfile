# Select the appropritae tag from pytorch docker hub to match your GPU
FROM pytorch/pytorch:2.1.0-cuda11.8-cudnn8-runtime

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
  curl \
  git \
  python3-pip \
  build-essential \
  libopencv-dev \
  python3-opencv

RUN pip install pycocotools matplotlib onnxruntime onnx opencv-python

RUN git clone https://github.com/facebookresearch/segment-anything.git
WORKDIR segment-anything
RUN pip install -e .

RUN mkdir weights

COPY sam_vit_h_4b8939.pth weights/sam_vit_h_4b8939.pth

# TODO @ChandimaFernando: Change the below file to a github repo
COPY sam_holder_mask.py sam_holder_mask.py
COPY sam_color_select_mask.py sam_color_select_mask.py
