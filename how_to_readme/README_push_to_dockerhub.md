[(return to main menu)](https://github.com/RedLeader962/Dockerized-SNOW)
## How-to push image localy builded image to docker hub from command line
```shell

sudo docker login --username norlabsnow --password <yourPass>

# create a new repo if necessary
sudo docker tag 
#or re-tag an existing local image 
docker tag <existing-image>[:<tag>] norlabsnow/<repo-name>[:<tag>]

sudo docker push norlabsnow/<repo-name>[:<optionalTagname>]
```
with e.g. `<repo-name>` = `gt-autorally/deploy`

