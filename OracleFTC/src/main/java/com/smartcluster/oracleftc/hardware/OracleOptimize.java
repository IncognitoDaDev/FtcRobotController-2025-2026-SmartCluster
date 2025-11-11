package com.smartcluster.oracleftc.hardware;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Documented
@Inherited
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface OracleOptimize {
    int maximumParallelCommands() default 8;

    boolean singleThreadOptimized() default true;

    boolean consistentLoopTimes() default false;
}
