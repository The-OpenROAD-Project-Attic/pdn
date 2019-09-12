echo "Building new pdn code within OpenROAD Flow"
echo "Building .."

cd ../openroad-builds/pdn/
git pull
git submodule update --remote
cd module/pdn
git checkout openroad-build
git pull
cd ../../
make build-tools