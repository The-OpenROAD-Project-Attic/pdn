pipeline {
  agent any
  stages {
    stage('Build') {
      steps {
        sh './jenkins/build.sh'
      }
    }
    stage('Test') {
      steps {
        sh './jenkins/test.sh'
      }
    }
    stage('OpenROAD Build') {
      when {
        branch 'openroad-build'
      }
      steps {
        sh './jenkins/openroad-build.sh'
      }
    }
    stage('OpenROAD Flow Test') {
      when {
        branch 'openroad-build'
      }
      parallel {
        stage('GCD - Nangate45') {
          steps {
            sh './jenkins/openroad-test.sh DESIGN_CONFIG=/openroad/flow/designs/gcd_nangate45.mk'
            }
        }
        stage('AES - Nangate45') {
          steps {
            sh './jenkins/openroad-test.sh DESIGN_CONFIG=/openroad/flow/designs/aes_nangate45.mk'
            }
        }
        stage('Dynamic Node - Nangate45') {
          steps {
            sh './jenkins/openroad-test.sh DESIGN_CONFIG=/openroad/flow/designs/dynamic_node_nangate45.mk'
            }
        }
        stage('IBEX - Nangate45') {
          steps {
            sh './jenkins/openroad-test.sh DESIGN_CONFIG=/openroad/flow/designs/ibex_nangate45.mk'
            }
        }
        stage('SWERV - Nangate45') {
          steps {
            sh './jenkins/openroad-test.sh DESIGN_CONFIG=/openroad/flow/designs/swerv_nangate45.mk'
            }
        }
      }
    }
  }
}
