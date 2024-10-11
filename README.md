
Test in Docker
==============

1. Clone repo and build image:

   ```
   git clone git@github.com:ana-GT/compose_setups.git -b reachability
   cd compose_setups
   ./scripts/build_images.sh
   ./scripts/clone_rosws.sh
   ./scripts/build_rosws.sh
   ```
   
2. Start services:

   ```
   docker compose -f docker-compose-dev.yml up
   ```
   
   and in a browser tab, open VNC so you can see GUIs: 
   ```
   http://localhost:8080/vnc.html
   ```
   
3. Open a terminal within the running container and now you can do your stuff here:

   ```
   docker exec -it compose_setups-rosws-1 bash
   source install/setup.bash
   
   ```
