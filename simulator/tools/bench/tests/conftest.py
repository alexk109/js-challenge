def pytest_configure(config):
    config.addinivalue_line("markers", "long: slow test (flow, RTL sweep, gate level, power); run by make bench-test-long")
