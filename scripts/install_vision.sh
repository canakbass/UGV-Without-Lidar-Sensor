#!/bin/bash
set -e

echo "============================================"
echo "  INSTALLING VISION DEPENDENCIES            "
echo "============================================"

pip3 install ultralytics easyocr opencv-python-headless

echo "============================================"
echo "  VISION DEPENDENCIES INSTALLED! ✅          "
echo "============================================"
