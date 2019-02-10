package jsonrpc.client.builder;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import jsonrpc.client.JsonRpcId;
import jsonrpc.client.JsonRpcParams;
import jsonrpc.client.ParamsType;
import jsonrpc.client.generator.AtomicLongIdGenerator;
import jsonrpc.client.generator.IdGenerator;
import jsonrpc.client.metadata.ClassMetadata;
import jsonrpc.client.metadata.MethodMetadata;
import jsonrpc.client.metadata.ParameterMetadata;
import jsonrpc.core.annotation.JsonRpcMethod;
import jsonrpc.core.annotation.JsonRpcOptional;
import jsonrpc.core.annotation.JsonRpcParam;
import jsonrpc.core.annotation.JsonRpcService;

import java.lang.annotation.Annotation;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Date: 11/17/14
 * Time: 8:04 PM
 * Utility class for gathering meta-information about client proxies through reflection
 *
 * @author Artem Prigoda
 */
class Reflections {

    private Reflections() {
    }

    /**
     * Gets remote service interface metadata
     *
     * @param clazz an interface for representing a remote service
     * @return class metadata
     */
    @NotNull
    public static ClassMetadata getClassMetadata(@NotNull Class<?> clazz) {
        Map<Method, MethodMetadata> methodsMetadata = new HashMap<Method, MethodMetadata>(32);
        Class<?> searchClass = clazz;
        while (searchClass != null) {
            JsonRpcService rpcServiceAnn = getAnnotation(searchClass.getAnnotations(), JsonRpcService.class);
            if (rpcServiceAnn == null) {
                throw new IllegalStateException("Class '" + clazz.getCanonicalName() +
                        "' is not annotated as @JsonRpcService");
            }
            Method[] methods = searchClass.getMethods();
            for (Method method : methods) {
                Annotation[] methodAnnotations = method.getDeclaredAnnotations();
                JsonRpcMethod rpcMethodAnn = getAnnotation(methodAnnotations, JsonRpcMethod.class);
                if (rpcMethodAnn == null) {
                    throw new IllegalStateException("Method '" + method.getName() + "' is not annotated as @JsonRpcMethod");
                }

                // LinkedHashMap is needed to support method parameter ordering
                Map<String, ParameterMetadata> paramsMetadata = new LinkedHashMap<String, ParameterMetadata>(8);
                Annotation[][] parametersAnnotations = method.getParameterAnnotations();
                for (int i = 0; i < parametersAnnotations.length; i++) {
                    Annotation[] parametersAnnotation = parametersAnnotations[i];
                    // Check that it's a JSON-RPC param
                    JsonRpcParam rpcParamAnn = getAnnotation(parametersAnnotation, JsonRpcParam.class);
                    if (rpcParamAnn == null) {
                        throw new IllegalStateException("Parameter with index=" + i + " of method '" + method.getName() +
                                "' is not annotated with @JsonRpcParam");
                    }
                    // Check that's a param could be an optional
                    JsonRpcOptional optionalAnn = getAnnotation(parametersAnnotation, JsonRpcOptional.class);
                    ParameterMetadata parameterMetadata = new ParameterMetadata(i, optionalAnn != null);
                    if (paramsMetadata.put(rpcParamAnn.value(), parameterMetadata) != null) {
                        throw new IllegalStateException("Two parameters of method '" + method.getName() + "' have the " +
                                "same name '" + rpcParamAnn.value() + "'");
                    }

                }
                String name = !rpcMethodAnn.value().isEmpty() ? rpcMethodAnn.value() : method.getName();
                ParamsType paramsType = getParamsType(methodAnnotations);
                methodsMetadata.put(method, new MethodMetadata(name, paramsType, paramsMetadata));
            }
            searchClass = searchClass.getSuperclass();
        }

        Annotation[] classAnnotations = clazz.getDeclaredAnnotations();
        IdGenerator<?> idGenerator = getIdGenerator(classAnnotations);
        ParamsType paramsType = getParamsType(classAnnotations);
        return new ClassMetadata(paramsType, idGenerator, methodsMetadata);
    }


    /**
     * Get an actual id generator
     */
    @NotNull
    private static IdGenerator<?> getIdGenerator(@NotNull Annotation[] classAnnotations) {
        JsonRpcId jsonRpcIdAnn = getAnnotation(classAnnotations, JsonRpcId.class);
        Class<? extends IdGenerator<?>> idGeneratorClazz = (jsonRpcIdAnn == null) ?
                AtomicLongIdGenerator.class : jsonRpcIdAnn.value();
        try {
            return idGeneratorClazz.newInstance();
        } catch (Exception e) {
            throw new IllegalStateException("Unable instantiate id generator: " + idGeneratorClazz, e);
        }
    }

    @Nullable
    private static ParamsType getParamsType(@NotNull Annotation[] annotations) {
        JsonRpcParams rpcParamsAnn = getAnnotation(annotations, JsonRpcParams.class);
        return rpcParamsAnn != null ? rpcParamsAnn.value() : null;

    }

    @SuppressWarnings("unchecked")
    @Nullable
    private static <T extends Annotation> T getAnnotation(@Nullable Annotation[] annotations,
                                                          @NotNull Class<T> clazz) {
        if (annotations != null) {
            for (Annotation annotation : annotations) {
                if (annotation.annotationType().equals(clazz)) {
                    return (T) annotation;
                }
            }
        }
        return null;
    }
}
