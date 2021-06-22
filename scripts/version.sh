#!/usr/bin/env bash

imageFile="$1"
version=$(grep -a "Firmware Version: " "${imageFile}" | grep -ao "[0-9]\+\\.[0-9]\+")
git_hash=$(git rev-parse --short HEAD)
full_version="${version}.${git_hash}"
echo "${full_version}"

