"""
Tests for package imports and structure
"""

import pytest


class TestPackageImports:
    """Tests for package-level imports"""
    
    def test_import_main_package(self):
        """Test importing main package"""
        import gsv8_ethercat
        
        assert gsv8_ethercat is not None
    
    def test_import_sensor(self):
        """Test importing sensor module"""
        from gsv8_ethercat import GSV8Sensor
        
        assert GSV8Sensor is not None
    
    def test_import_data(self):
        """Test importing GSV8Data"""
        from gsv8_ethercat import GSV8Data
        
        assert GSV8Data is not None
    
    def test_import_diagnostics(self):
        """Test importing diagnostics module"""
        from gsv8_ethercat import GSV8Diagnostics
        
        assert GSV8Diagnostics is not None
    
    def test_import_calibration(self):
        """Test importing calibration module"""
        from gsv8_ethercat import GSV8Calibration
        
        assert GSV8Calibration is not None
    
    def test_import_monitor(self):
        """Test importing monitor module"""
        from gsv8_ethercat import GSV8Monitor
        
        assert GSV8Monitor is not None
    
    def test_version_attribute(self):
        """Test that version attribute exists"""
        from gsv8_ethercat import __version__
        
        assert __version__ is not None
        assert isinstance(__version__, str)
        assert len(__version__) > 0
    
    def test_all_exports(self):
        """Test __all__ exports"""
        import gsv8_ethercat
        
        # Check that __all__ exists and contains expected exports
        assert hasattr(gsv8_ethercat, '__all__')
        assert 'GSV8Sensor' in gsv8_ethercat.__all__
        assert 'GSV8Data' in gsv8_ethercat.__all__
        assert 'GSV8Diagnostics' in gsv8_ethercat.__all__
        assert 'GSV8Calibration' in gsv8_ethercat.__all__
        assert 'GSV8Monitor' in gsv8_ethercat.__all__


class TestPackageMetadata:
    """Tests for package metadata"""
    
    def test_author_attribute(self):
        """Test author attribute"""
        from gsv8_ethercat import __author__
        
        assert __author__ is not None
        assert isinstance(__author__, str)
    
    def test_license_attribute(self):
        """Test license attribute"""
        from gsv8_ethercat import __license__
        
        assert __license__ is not None
        assert isinstance(__license__, str)
    
    def test_package_docstring(self):
        """Test package has docstring"""
        import gsv8_ethercat
        
        assert gsv8_ethercat.__doc__ is not None
        assert len(gsv8_ethercat.__doc__) > 0


class TestModuleStructure:
    """Tests for module structure"""
    
    def test_sensor_module_exists(self):
        """Test sensor module exists"""
        from gsv8_ethercat import sensor
        
        assert sensor is not None
    
    def test_diagnostics_module_exists(self):
        """Test diagnostics module exists"""
        from gsv8_ethercat import diagnostics
        
        assert diagnostics is not None
    
    def test_calibration_module_exists(self):
        """Test calibration module exists"""
        from gsv8_ethercat import calibration
        
        assert calibration is not None
    
    def test_monitor_module_exists(self):
        """Test monitor module exists"""
        from gsv8_ethercat import monitor
        
        assert monitor is not None
