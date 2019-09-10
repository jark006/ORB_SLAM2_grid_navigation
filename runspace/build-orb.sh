#!/bin/bash


cd ../build/

cmake .. -DROS_BUILD_TYPE=Release

make -j8

