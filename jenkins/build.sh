docker build --target base -t pdn .
docker run -v $(pwd):/pdn pdn bash -c "./pdn/jenkins/install.sh"
