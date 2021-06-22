#!/bin/bash
img="$1"
name="${img%.*}"
imgPath="release/app/run/$img"
curl -v -f -u $NEXUS_USER:$NEXUS_PASSWORD --upload-file "${imgPath}" https://tigers-mannheim.de/nexus/repository/firmware/edu/tigers/firmware/${name}/$(./scripts/version.sh ${imgPath})/${img}
