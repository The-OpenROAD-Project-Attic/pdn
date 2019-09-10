docker build -f jenkins/Dockerfile.dev -t pdn .
docker run -v $(pwd):/pdn pdn bash -c "./pdn/jenkins/install.sh"