This Docker container builds [SAM](https://github.com/facebookresearch/segment-anything) image segmentation algorithm. 

To run the docker container with a local mapping of the volume to read input images, run:
```bash
docker run --network host --ipc=host --gpus all -v /home/wfernando1/Documents/Environments/Workspaces/minimal_erobs_fork/erobs/data/:/workspace/data -it image-mask-fastsam:latest
```

When inside the container, execute the automatic segmentation algorithm by running :
```bash
python3 scripts/amg.py --checkpoint weights/sam_vit_h_4b8939.pth --model-type vit_h --input ../data/rgb/1713301695.png --output ../data/mask/ --min-mask-region-area 100
```