using Test
using Aqua
using sts2levelqs

#
@testset verbose = true showtiming = true "sts2levelqs tests" begin
    for name in (:aqua, :default)
        @testset "$(name)" begin
            test_name = Symbol(:test_, name)
            include("$(test_name).jl")
            @eval $test_name()
        end
    end
end
