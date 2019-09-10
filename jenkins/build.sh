docker build -f jenkins/Dockerfile.dev -t openroad/pdn .
docker run -v $(pwd):/pdn openroad/pdn bash -c "./pdn/jenkins/install.sh"