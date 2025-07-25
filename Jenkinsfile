#!/usr/bin/env groovy

pipeline {

    agent {
        docker {
            image 'lsstts/rust:latest'
        }
    }

    triggers {
        pollSCM('H * * * *')
    }

    environment {
        // XML report path
        XML_REPORT_COVERAGE = "coverage.xml"
    }

    options {
        disableConcurrentBuilds()
    }

    stages {

        stage('Check Code') {
            steps {
                // 'PATH' can only be updated in a single shell block.
                // We can not update PATH in 'environment' block.
                withEnv(["HOME=${env.WORKSPACE}"]) {
                    sh """
                        cargo check
                        cargo test check
                    """
                }
            }
        }

        stage('Unit Tests and Code Coverage') {
            steps {
                // 'PATH' can only be updated in a single shell block.
                // We can not update PATH in 'environment' block.
                withEnv(["HOME=${env.WORKSPACE}"]) {
                    sh """
                        cargo llvm-cov nextest --cobertura --output-path ${env.XML_REPORT_COVERAGE} --profile ci
                    """
                }
            }
        }
    }

    post {
        always {
            // The path of xml needed by xUnit is relative to
            // the workspace.
            xunit (
                thresholds: [ skipped(failureThreshold: '0'), failed(failureThreshold: '0') ],
                tools: [ JUnit(pattern: 'target/nextest/ci/*.xml') ]
            )

            // Publish the coverage report
            recordCoverage(
                tools: [[parser: 'COBERTURA', pattern: '*.xml']]
            )
        }

        cleanup {
            // clean up the workspace
            deleteDir()
        }
    }
}
