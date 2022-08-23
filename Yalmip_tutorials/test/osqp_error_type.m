classdef osqp_error_type < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        OSQP_DATA_VALIDATION_ERROR(1),
		OSQP_SETTINGS_VALIDATION_ERROR(2),
		OSQP_LINSYS_SOLVER_LOAD_ERROR(3),
		OSQP_LINSYS_SOLVER_INIT_ERROR(4),
		OSQP_NONCVX_ERROR(5),
		OSQP_MEM_ALLOC_ERROR(6),
		OSQP_WORKSPACE_NOT_INIT_ERROR(7)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = osqp_error_type.OSQP_DATA_VALIDATION_ERROR;
        end

        function dScope = getDataScope()
            % GETDATASCOPE  Specifies whether the data type definition should be imported from,
            %               or exported to, a header file during code generation.
            dScope = 'Imported';
        end

        function desc = getDescription()
            % GETDESCRIPTION  Returns a description of the enumeration.
            desc = '';
        end
        
        function headerFile = getHeaderFile()
            % GETHEADERFILE  Specifies the name of a header file. 
            headerFile = 'sfun_header.h';
        end
        
        function flag = addClassNameToEnumNames()
            % ADDCLASSNAMETOENUMNAMES  Indicate whether code generator applies the class name as a prefix
            %                          to the enumeration.
            flag = false;
        end

    end

end
