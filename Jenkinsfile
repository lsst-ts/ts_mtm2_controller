#!/usr/bin/env groovy

pipeline {

    agent {
        docker {
            image 'lsstts/rust:develop'
        }
    }

    triggers {
        pollSCM('H * * * *')
    }

    environment {
        // Report path
        REPORT_COVERAGE = "coverage.info"
        REPORT_JUNIT = "junit.xml"
    }

    options {
        disableConcurrentBuilds()
    }

    stages {

        stage('Linting Code') {
            steps {
                // 'PATH' can only be updated in a single shell block.
                // We can not update PATH in 'environment' block.
                withEnv(["HOME=${env.WORKSPACE}"]) {
                    sh """
                        cargo clippy
                    """
                }
            }
        }

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
                        cargo llvm-cov nextest --no-cfg-coverage --lcov --output-path ${env.REPORT_COVERAGE} --profile ci
                        sed -i 's/ uuid=\"[^\"]*\"//g' target/nextest/ci/${env.REPORT_JUNIT}
                        sed -i 's/ timestamp=\"[^\"]*\"//g' target/nextest/ci/${env.REPORT_JUNIT}
                        sed -i 's/ disabled=\"[^\"]*\"//g' target/nextest/ci/${env.REPORT_JUNIT}
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
                tools: [[parser: 'LCOV', pattern: '*.info']]
            )
        }

        cleanup {
            // clean up the workspace
            deleteDir()
        }
    }
}
