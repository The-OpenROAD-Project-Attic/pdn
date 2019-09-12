echo "Testing new pdn code on OpenROAD Flow"
echo "Testing .."

cd ../openroad-builds/pdn
docker run -v $(pwd):/openroad openroad/flow bash -c "cd /openroad/flow && make clean_all && make DESIGN_CONFIG=$1"
