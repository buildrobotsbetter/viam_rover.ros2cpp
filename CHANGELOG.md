# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## [0.1.0] - 2024-08-27

### Added
- Initial support
    - Drive control (forward, backward, rotate left, rotate right) using `/cmd_vel` topic
    - Timeout is no commands are received within 2 seconds
    - Uses libgpio v0.2.0
