FROM openroad/opendb:dev as base

ENV PATH=/pdn/module/OpenDB/build/src/swig/tcl:$PATH
ENV TCLLIBPATH="/pdn/src/scripts $TCLLIBPATH"

# Build
FROM base AS builder
COPY . /pdn
RUN sh /pdn/jenkins/install.sh

# Runner
FROM centos:centos7 AS runner
RUN yum update -y && yum install -y perl
COPY --from=builder /pdn/src/scripts /build/src/scripts/
COPY --from=builder /pdn/test /build/test/
ENV PATH=/pdn/module/OpenDB/build/src/swig/tcl:/build:/build/scripts/:$PATH \
    TCLLIBPATH="/build/src/scripts $TCLLIBPATH"
WORKDIR /build/test
WORKDIR /pdn

