package com.smartcluster.oracleftc.utils;

import com.qualcomm.robotcore.util.RobotLog;
import java.lang.reflect.Field;

/**
 * Reflection utilities for library or user use
 */
public class ReflectionUtils {

    /**
     * Gets the field of a class
     *
     * @param clazz     class to which the field belongs
     * @param fieldName name of the field
     * @return the field
     */
    public static Field getField(Class<?> clazz, String fieldName) {
        try {
            Field field = clazz.getDeclaredField(fieldName);
            field.setAccessible(true);
            return field;
        } catch (NoSuchFieldException e) {
            Class<?> superClass = clazz.getSuperclass();
            if (superClass != null) {
                return getField(superClass, fieldName);
            }
        }
        return null;
    }

    /**
     * Gets the value of a field
     *
     * @param object    object to which the field belongs
     * @param fieldName name of the field
     * @param <T>       type of the field
     * @return value of the field
     */
    @SuppressWarnings({"unchecked"})
    public static <T> T getFieldValue(Object object, String fieldName) {
        Field field = getField(object.getClass(), fieldName);
        if (field != null) {
            field.setAccessible(true);
            try {
                return (T) field.get(object);
            } catch (IllegalAccessException e) {
                RobotLog.logStackTrace(e);
            }
        }
        return null;
    }

    /**
     * Sets the value of a field
     *
     * @param object    object to which the field belongs
     * @param fieldName name of the field
     * @param value     new value of the field
     */
    @SuppressWarnings({"unused"})
    public static void setFieldValue(Object object, String fieldName, Object value) {
        Field field = getField(object.getClass(), fieldName);
        if (field != null) {
            field.setAccessible(true);
            try {
                field.set(object, value);
            } catch (Exception e) {
                RobotLog.logStackTrace(e);
            }
        }
    }

    /**
     * Copies all values from a object to a target
     *
     * @param org    object
     * @param target target
     */
    @SuppressWarnings({"CallToPrintStackTrace"})
    public static void deepCopy(Object org, Object target) {
        Field[] fields = org.getClass().getDeclaredFields();
        for (Field f : fields) {
            f.setAccessible(true);
            Field f2 = getField(target.getClass(), f.getName());
            if (f2 != null) {
                f2.setAccessible(true);
                try {
                    f2.set(target, f.get(org));
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
