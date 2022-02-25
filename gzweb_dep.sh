#!/bin/bash

# install nvm
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash

source ~/.nvm/nvm.sh

# install node. Supported versions are 8 to 11. 
nvm install 8
