FROM openroad/opendb:dev as base

ENV TCLLIBPATH="/pdn/src/scripts $TCLLIBPATH"
ENV PATH=/pdn/module/OpenDB/build/src/swig/tcl:$PATH

# Build
FROM base AS builder
COPY . /pdn
RUN sh /pdn/jenkins/install.sh

# Runner
FROM centos:centos7 AS runner
RUN yum update -y && yum install -y perl
COPY --from=builder /pdn/src/scripts /build/scripts/
COPY --from=builder /pdn/test /build/test/
ENV PATH=/pdn/module/OpenDB/build/src/swig/tcl:/build:/build/scripts/:$PATH \
    TCLLIBPATH="/build/scripts $TCLLIBPATH"
WORKDIR /pdn
