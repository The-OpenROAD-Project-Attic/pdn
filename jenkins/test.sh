docker run -v $(pwd)/test:/test pdn bash -c "cd /test; chmod u+rwx .; make check"
