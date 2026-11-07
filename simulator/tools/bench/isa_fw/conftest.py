def pytest_configure(config):
    config.addinivalue_line("markers", "long: slow test (runs a whole fw suite); make bench-test-isa-long")
