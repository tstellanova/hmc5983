; ModuleID = 'probe3.b9606731-cgu.0'
source_filename = "probe3.b9606731-cgu.0"
target datalayout = "e-m:e-p:32:32-Fi8-i64:64-v128:64:128-a:0:32-n32-S64"
target triple = "thumbv7em-none-unknown-eabihf"

; core::f64::<impl f64>::to_int_unchecked
; Function Attrs: inlinehint nounwind
define dso_local i32 @"_ZN4core3f6421_$LT$impl$u20$f64$GT$16to_int_unchecked17h2578ca5f984c8cbcE"(double %self) unnamed_addr #0 {
start:
; call <f64 as core::convert::num::FloatToInt<i32>>::to_int_unchecked
  %0 = call i32 @"_ZN65_$LT$f64$u20$as$u20$core..convert..num..FloatToInt$LT$i32$GT$$GT$16to_int_unchecked17hd846f3fc6fef6a9aE"(double %self) #2
  br label %bb1

bb1:                                              ; preds = %start
  ret i32 %0
}

; <f64 as core::convert::num::FloatToInt<i32>>::to_int_unchecked
; Function Attrs: inlinehint nounwind
define internal i32 @"_ZN65_$LT$f64$u20$as$u20$core..convert..num..FloatToInt$LT$i32$GT$$GT$16to_int_unchecked17hd846f3fc6fef6a9aE"(double %self) unnamed_addr #0 {
start:
  %0 = alloca i32, align 4
  %1 = fptosi double %self to i32
  store i32 %1, i32* %0, align 4
  %2 = load i32, i32* %0, align 4
  br label %bb1

bb1:                                              ; preds = %start
  ret i32 %2
}

; probe3::probe
; Function Attrs: nounwind
define dso_local void @_ZN6probe35probe17h838fac25c1c1e963E() unnamed_addr #1 {
start:
; call core::f64::<impl f64>::to_int_unchecked
  %_1 = call i32 @"_ZN4core3f6421_$LT$impl$u20$f64$GT$16to_int_unchecked17h2578ca5f984c8cbcE"(double 1.000000e+00) #2
  br label %bb1

bb1:                                              ; preds = %start
  ret void
}

attributes #0 = { inlinehint nounwind "frame-pointer"="all" "target-cpu"="generic" "target-features"="+vfp4,-d32,-fp64" }
attributes #1 = { nounwind "frame-pointer"="all" "target-cpu"="generic" "target-features"="+vfp4,-d32,-fp64" }
attributes #2 = { nounwind }
